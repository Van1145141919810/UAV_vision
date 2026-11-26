# -*- coding: utf-8 -*-
"""window_finder.py —— 三路融合的「窗户定位器」

三路各管一段（详见 docs/04_多方法窗户定位_方案与实验.md）
  ① dot      项目自带红点检测（文豪的 vb09.perception.reddot）—— 快，0.03 s，只认颜色
  ② seg      CMP-facade SegFormer（14 MB）—— 把窗户一扇一扇描出来，0.4 s
  ③ gdino    Grounding DINO tiny（零样本）—— 用文字提示理解"这是窗户/玻璃门"，2.7 s

融合规则（先定死，实验后再调）
  1. 红点 + 有窗户框把它包住  → 最可信：target=那个窗户框，anchor=红点
  2. 只有红点              → target=红点框，提示"只见红点，未见窗户"
  3. 只有窗户框            → 取离画面中心最近的窗户框，提示"只见窗户，未见红点"
  4. 都没有                → ok=False，界面提示"看不清，请你手动挪一下"

输出（一个 dict，给控制台/叠加层直接吃）
  {ok, source, box:[x1,y1,x2,y2], center:[cx,cy],
   dist_m, height_m, floor, dot:{...}|None, win:{...}|None, text, ms:{...}}

依赖
  · 本模块跑在 **mcpv** 环境（cv2/numpy 就够）
  · seg / gdino 由 venv_det 里的 scripts/winseg_worker.py 子进程提供（stdin/stdout JSON）
  · 红点检测需要 _ref/s14 在 sys.path；拿不到就自动降级成只用 seg/gdino

用法
    from mcpv.window_finder import WindowFinder
    wf = WindowFinder()            # 第一次调用会 spawn worker（GDINO 加载约 7 s）
    r = wf.find(bgr)               # bgr: BGR uint8
    print(r["text"], r["box"])
"""
from __future__ import annotations

import base64
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]          # …/MiniCPM-V
S14 = Path(r"D:\Desktop\无人系统\_ref\s14")
WORKER_PY = ROOT / "scripts" / "winseg_worker.py"
VENV_PY = ROOT / "venv_det" / "Scripts" / "python.exe"

# ── 物理参数（改这里）──────────────────────────────────────────────
FX = 1626.6183          # 1920 校正左目焦距
DOT_D_M = 0.20          # 红圆直径 (m)
WIN_W_M = 1.50          # 一扇窗户名义宽度 (m)；居民楼常见 1.2~1.8
CAM_H_M = 1.50          # 相机离地高度 (m)，起飞高度
GDINO_PROMPT = "window. glass window. glass door. curtain wall."
GDINO_SHORT = 400       # 输入短边：400→约 2.7 s（实测精度基本不掉）

# 红点直径标定系数（2026-11-23 实测 21 帧）
#   我量的红色连通域等效直径 比 几何直径 系统性偏大（因为边缘被模糊/压缩放大）
#   中位数 k = 0.863（范围 0.58~1.03）→ 单帧距离误差约 ±20%
DOT_DIAM_K = 0.86
DIST_ERR_PCT = 20       # 红点测距的名义误差（写进输出，让界面能显示不信任度）

# ── 语义闸门（防"红花盆当红点"）────────────────────────────────────
# 2026-11-24 实测（5 正例 / 10 负例，负例全是红点检测器在白天立面照上的误报）：
#   · 只靠 VLM 问"圆点还是杂物"      → 正例 5/5 保住，负例只排除 4/10（漏 60%）
#   · 只靠物理白环 nw>=4            → 正例 4/5，负例排除 7/10
#   · 两者取与（nw>=4 且 VLM 说"圆点"）→ **正例 4/5，负例排除 9/10**
# 所以闸门 = 物理优先 + 语义兜底，不是纯 VLM。
NW_MIN = 4              # 白环扇区下限（0~8）
NW_MIN_DPX = 20         # 只有点径 >= 20 px 时白环才可信；再小就是亚像素，不能用它判死
VLM_GATE = True         # 有红点时是否再问一次 MiniCPM-V
VLM_URL = "http://127.0.0.1:11434/api/generate"
VLM_MODEL = "openbmb/minicpm-v4.7:1b"
VLM_Q = ("画面正中心那个红色东西是【一个人为画的圆形圆点】，还是"
         "【花、砖、衣服、招牌之类的杂物】？只回答一个字母：A=圆点，B=杂物。")

# 融合阈值
DOT_TO_WIN_TOL_PX = 25      # 红点离窗框边多少像素内算"在窗上"
WIN_MIN_SIDE_PX = 30        # 太小的窗户框丢掉
GDINO_MIN_SCORE = 0.22

# 标签优先级：零样本的提示词里 "curtain wall"（幕墙）会被**织物窗帘**命中，
#   2026-11-24 实测：室内对着窗帘，GDINO 给了 curtain wall 0.664（最高分），
#   把真正的 glass window 0.326 压下去了 → 融合选错。
#   所以按"这个词有多像我想要的目标"再乘一个权重。
LABEL_PRIO = (
    ("glass window", 1.00),
    ("window", 0.95),
    ("glass door", 0.80),
    ("glass", 0.70),
    ("curtain wall", 0.40),      # 可能就是窗帘
    ("facade", 0.20),
)


def label_prio(lab):
    s = str(lab or "").lower()
    for key, w in LABEL_PRIO:
        if key in s:
            return w
    return 0.50


# ══════════════════════════════════════════════════════════════════
# worker 客户端（常驻子进程，模型只加载一次）
# ══════════════════════════════════════════════════════════════════
class _Worker(object):
    def __init__(self):
        self.p = None
        self.lock = threading.Lock()
        self.n = 0
        self.err = None

    def _spawn(self):
        if not VENV_PY.exists():
            raise RuntimeError("找不到 venv_det：%s" % VENV_PY)
        # 先清掉上一次残留的 worker —— 控制台窗口被直接关掉时子进程会活下来，
        # 两个 worker 同时加载模型会抢 CPU（2026-11-24 实测预热翻倍）。
        # 用 worker 自己写的 pid 文件精确杀，比 PowerShell 扫全表快很多。
        try:
            pidf = Path(os.environ.get("TEMP", ".")) / "cpmv_winseg.pid"
            if pidf.exists():
                old = int(pidf.read_text().strip() or 0)
                if old > 0:
                    try:
                        os.kill(old, 9)
                    except Exception:
                        pass
                pidf.unlink()
        except Exception:
            pass
        env = dict(os.environ)
        env.setdefault("HF_HOME", str(ROOT / ".hf"))
        env.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
        # ★ 2026-11-24：两个模型都已经缓存在 .hf 里，**必须离线加载**。
        #   否则 from_pretrained 会对每个文件发 HEAD 去问 hf-mirror，
        #   网络不通时每个都等超时 —— 实测预热从 21 s 变成 44 s。
        env["HF_HUB_OFFLINE"] = "1"
        env["TRANSFORMERS_OFFLINE"] = "1"
        env["HF_HUB_ETAG_TIMEOUT"] = "2"
        env["PYTHONIOENCODING"] = "utf-8"
        env["TRANSFORMERS_VERBOSITY"] = "error"
        self.p = subprocess.Popen(
            [str(VENV_PY), str(WORKER_PY)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            cwd=str(ROOT), env=env, text=True, encoding="utf-8", bufsize=1)
        return self.p

    def call(self, obj, timeout=120.0):
        with self.lock:
            if self.p is None or self.p.poll() is not None:
                self._spawn()
            self.n += 1
            obj["id"] = self.n
            self.p.stdin.write(json.dumps(obj, ensure_ascii=True) + "\n")
            self.p.stdin.flush()
            line = self.p.stdout.readline()
            if not line:
                self.err = "worker 没回话（可能崩了）"
                raise RuntimeError(self.err)
            return json.loads(line)

    def close(self):
        try:
            if self.p and self.p.poll() is None:
                self.call({"cmd": "quit"}, timeout=5)
        except Exception:
            pass
        try:
            if self.p:
                self.p.kill()
        except Exception:
            pass
        self.p = None


# ══════════════════════════════════════════════════════════════════
# 红点检测（复用项目自己的检测器）
# ══════════════════════════════════════════════════════════════════
class _DotSource(object):
    def __init__(self, rig):
        self.ok = False
        self.rig = rig
        self.det = None
        self.cfg = None
        try:
            if str(S14) not in sys.path:
                sys.path.insert(0, str(S14))
            from vb09.perception.reddot.config import DotConfig
            from vb09.perception.reddot.detect import DotDetectorCore
            self.cfg = DotConfig()
            # 2026-11-23 实测：36~45 m 上 min_d_px=7.0 / open_k=3 最好（25/25）
            self.cfg.min_d_px, self.cfg.dot_d, self.cfg.d_max, self.cfg.open_k = 7.0, 0.20, 0.28, 3
            self.det = DotDetectorCore(rig, self.cfg)
            self.ok = True
        except Exception as e:
            self.err = "%s: %s" % (type(e).__name__, e)

    def find(self, L):
        """返回 [{u,v,d_px,score}]，d_px 用红色连通域等效直径（不是 dmaj）。"""
        if not self.ok:
            return []
        t0 = time.time()
        try:
            out = self.det.process(L, L, time.time())
        except Exception:
            return []
        ms = (time.time() - t0) * 1000
        res = []
        for c in (getattr(out, "cands", None) or []):
            u, v = int(c.u), int(c.v)
            d = self._red_diam(L, u, v)
            if d <= 0:
                d = float(getattr(c, "dmaj", 0) or 0) * 0.55   # 兜底：粗略换算
            res.append(dict(u=u, v=v, d_px=round(float(d), 2),
                            score=float(getattr(c, "score", 0) or 0),
                            nw=int(getattr(c, "nw", -1)),          # 白环扇区数 0~8
                            fill=float(getattr(c, "fill", 0) or 0),
                            circ=float(getattr(c, "circ", 0) or 0),
                            axr=float(getattr(c, "axr", 0) or 0)))
        self.ms = ms
        return res

    @staticmethod
    def _red_diam(L, u, v, win=26):
        x0, y0 = max(0, u - win), max(0, v - win)
        sub = L[y0:y0 + 2 * win, x0:x0 + 2 * win]
        if sub.size == 0:
            return 0.0
        b, g, r = (sub[:, :, 0].astype(np.int16), sub[:, :, 1].astype(np.int16),
                   sub[:, :, 2].astype(np.int16))
        m = ((r - np.maximum(g, b)) > 45).astype(np.uint8)
        n, _, st, _ = cv2.connectedComponentsWithStats(m, 8)
        if n <= 1:
            return 0.0
        return float(2.0 * math.sqrt(max(st[1:, 4]) / math.pi))


# ══════════════════════════════════════════════════════════════════
# 语义闸门：把红点周围裁一小块，问 MiniCPM-V "圆点还是杂物"
# ══════════════════════════════════════════════════════════════════
def _vlm_ask_crop(crop, prompt=VLM_Q, npd=4, timeout=30):
    """返回 ('A'|'B'|'?', 耗时 ms)。异常一律返回 '?'，绝不抛。"""
    import base64
    import urllib.request
    t0 = time.time()
    try:
        ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            return "?", 0.0
        body = json.dumps({"model": VLM_MODEL, "prompt": prompt,
                           "images": [base64.b64encode(buf.tobytes()).decode()],
                           "stream": False, "think": False, "keep_alive": "30m",
                           "options": {"temperature": 0, "num_predict": npd}}).encode()
        rq = urllib.request.Request(VLM_URL, data=body,
                                    headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(rq, timeout=timeout) as r:
            txt = (json.loads(r.read()).get("response") or "").strip().upper()
        for ch in txt[:8]:
            if ch in "AB":
                return ch, (time.time() - t0) * 1000
    except Exception:
        pass
    return "?", (time.time() - t0) * 1000


def vlm_gate(img, dot, k=6.0, min_side=96, out=256):
    """裁红点周围一块问 VLM。返回 dict(verdict, ms, crop)。"""
    u, v, d = int(dot["u"]), int(dot["v"]), max(10.0, float(dot.get("d_px") or 10))
    side = int(max(min_side, d * k))
    h, w = img.shape[:2]
    x0 = int(max(0, min(w - side, u - side // 2)))
    y0 = int(max(0, min(h - side, v - side // 2)))
    c = img[y0:y0 + side, x0:x0 + side]
    if c.size == 0:
        return {"verdict": "?", "ms": 0.0}
    c = cv2.resize(c, (out, out), interpolation=cv2.INTER_CUBIC)
    ans, ms = _vlm_ask_crop(c)
    return {"verdict": ans, "ms": round(ms, 1)}


# ══════════════════════════════════════════════════════════════════
# 主类
# ══════════════════════════════════════════════════════════════════
class WindowFinder(object):
    def __init__(self, rig=None, use_dot=True, use_seg=True, use_gdino=True):
        self.use_dot, self.use_seg, self.use_gdino = use_dot, use_seg, use_gdino
        self.worker = _Worker()
        self.dot = None
        if use_dot:
            if rig is None:
                try:
                    if str(S14) not in sys.path:
                        sys.path.insert(0, str(S14))
                    from vb09.perception.stereo_rig import StereoRig
                    rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
                except Exception as e:
                    print("[window_finder] 载入标定失败，红点路关闭: %s" % e, flush=True)
                    rig = None
            if rig is not None:
                self.dot = _DotSource(rig)
                if not self.dot.ok:
                    print("[window_finder] 红点检测不可用: %s" % getattr(self.dot, "err", "?"), flush=True)
        # 后台预热：把 worker 拉起来 + 两个模型加载进内存（第一次约 45 s，之后每帧只花推理时间）
        self._warm = threading.Thread(target=self._warmup, daemon=True)
        self._warm.start()

    def _warmup(self):
        try:
            self.worker.call({"cmd": "ping"}, timeout=180)
            dummy = np.zeros((360, 640, 3), np.uint8)
            ok, buf = cv2.imencode(".jpg", dummy)
            self.worker.call({"cmd": "detect", "img": base64.b64encode(buf.tobytes()).decode(),
                              "seg": bool(self.use_seg), "gdino": bool(self.use_gdino),
                              "short": 320, "prompts": "window."}, timeout=300)
            # 顺手把 VLM（Ollama）也热起来 —— 否则第一次按 I 要多等 4~6 s 冷启
            if VLM_GATE:
                _vlm_ask_crop(dummy, npd=2, timeout=120)
            self.warm_ok = True
        except Exception as e:
            self.warm_ok = False
            self.warm_err = "%s: %s" % (type(e).__name__, e)

    def wait_ready(self, timeout=180.0):
        """等预热完成（演示/实验前调一次）。返回 True=两个模型都热了。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if not self._warm.is_alive():
                return bool(getattr(self, "warm_ok", False))
            time.sleep(0.2)
        return False

    # ---------------------------------------------------------------- 工具
    @staticmethod
    def _inside(box, u, v, tol=0):
        return (box[0] - tol <= u <= box[0] + box[2] + tol and
                box[1] - tol <= v <= box[1] + box[3] + tol)

    @staticmethod
    def _dist_from_box(box, u, v):
        """点到框的最近距离（在框里为 0）。box=[x,y,w,h]"""
        dx = max(box[0] - u, 0, u - (box[0] + box[2]))
        dy = max(box[1] - v, 0, v - (box[1] + box[3]))
        return math.hypot(dx, dy)

    def _height_m(self, cy, h, dist_m):
        """按仰角算高度：画面中心以上为正。"""
        if dist_m is None:
            return None
        ang = math.atan2((h / 2.0 - cy), FX)
        return round(CAM_H_M + dist_m * math.tan(ang), 2)

    # ---------------------------------------------------------------- 主入口
    def find(self, bgr, prompts=None, short=None, want_mask=False):
        t_all = time.time()
        h, w = bgr.shape[:2]
        ms = {}
        r = {"ok": False, "source": "none", "box": None, "center": None,
             "dist_m": None, "height_m": None, "floor": None,
             "dot": None, "win": None, "text": "", "ms": ms, "w": w, "h": h}

        # ① 红点
        dots = []
        if self.dot is not None and self.dot.ok:
            L = bgr
            try:
                if str(S14) not in sys.path:
                    sys.path.insert(0, str(S14))
                from vb09.perception.stereo_rig import StereoRig  # noqa: F401
                L, _ = self.dot.rig.rectify(bgr, bgr)
            except Exception:
                L = bgr
            dots = self.dot.find(L)
            ms["dot"] = round(getattr(self.dot, "ms", 0.0), 1)
            if dots:
                dots.sort(key=lambda d: math.hypot(d["u"] - w / 2.0, d["v"] - h / 2.0))
                r["dot"] = dict(dots[0])
                r["dot"]["n"] = len(dots)

        # ②③ 窗户（worker）
        segblobs, gdboxes = [], []
        if self.use_seg or self.use_gdino:
            try:
                ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                resp = self.worker.call({"cmd": "detect",
                                         "img": base64.b64encode(buf.tobytes()).decode(),
                                         "seg": bool(self.use_seg), "gdino": bool(self.use_gdino),
                                         "short": int(short or GDINO_SHORT),
                                         "prompts": prompts or GDINO_PROMPT})
                if resp.get("ok"):
                    if self.use_seg and "seg" in resp:
                        segblobs = resp["seg"].get("blobs") or []
                        ms["seg"] = resp["seg"].get("ms")
                        if want_mask and resp["seg"].get("mask"):
                            r["seg_mask_png"] = resp["seg"]["mask"]
                    if self.use_gdino and "gdino" in resp:
                        gdboxes = resp["gdino"].get("boxes") or []
                        ms["gdino"] = resp["gdino"].get("ms")
                else:
                    ms["worker_err"] = resp.get("err")
            except Exception as e:
                ms["worker_err"] = "%s: %s" % (type(e).__name__, e)

        # 候选窗户框：[x,y,w,h,src,score]
        r["gd_boxes"] = [b for b in gdboxes if len(b) >= 5]
        r["seg_blobs"] = segblobs
        cands = []
        for b in gdboxes:
            x1, y1, x2, y2, sc, lab = b
            bw, bh = x2 - x1, y2 - y1
            if bw < WIN_MIN_SIDE_PX or bh < WIN_MIN_SIDE_PX or sc < GDINO_MIN_SCORE:
                continue
            # 排序权重 = 分数 × 标签优先级（防 "curtain wall" 命中真窗帘）
            cands.append([x1, y1, bw, bh, "gdino", sc, lab, sc * label_prio(lab)])
        for (x, y, bw, bh, a) in segblobs:
            if bw < WIN_MIN_SIDE_PX or bh < WIN_MIN_SIDE_PX:
                continue
            cands.append([float(x), float(y), float(bw), float(bh), "seg", 0.30, "window",
                          0.30 * 0.95])

        # ── 语义闸门：有红点时判"这真是圆形标记吗" ─────────────────
        #   闸门 = 物理白环(nw) 优先 + VLM 兜底；只降级，不删框。
        #   实测（5 正例/10 负例）：nw>=4 单独 4/5 保正、3/10 误放；
        #   叠加 VLM(Q3) 后 4/5 保正、1/10 误放。
        _dg = r.get("dot")
        if _dg is not None:
            nw = int(_dg.get("nw", -1))
            dpx = float(_dg.get("d_px") or 0.0)
            gate = {"nw": nw, "nw_min": NW_MIN, "d_px": round(dpx, 1), "vlm": None, "ms": 0.0}
            why = []
            # ① 物理闸门：**只在点够大时**才算数
            #    41 m 时点只有 12 px，1.3~1.8R 的白环是亚像素 → nw 恒为 0，
            #    拿它判死会把真目标误杀。所以 d_px < NW_MIN_DPX 时跳过物理判据。
            if dpx >= NW_MIN_DPX and 0 <= nw < NW_MIN:
                why.append("白环不足 nw=%d<%d" % (nw, NW_MIN))
            # ② 语义闸门：只要有红点就问一次（约 1~1.5 s）
            if VLM_GATE:
                g = vlm_gate(bgr, _dg)
                gate["vlm"] = g["verdict"]
                gate["ms"] = g["ms"]
                if g["verdict"] == "B":
                    why.append("VLM 说不是圆点")
                elif g["verdict"] == "?":
                    why.append("VLM 没答上来")
            gate["passed"] = not why
            gate["why"] = "；".join(why) if why else "通过"
            r["gate"] = gate
            ms["vlm_gate"] = gate["ms"]
            if not gate["passed"]:
                r["gate_failed"] = True
                r["gate_note"] = "红点可疑：%s" % gate["why"]

        # ── 融合 ───────────────────────────────────────────────────
        dot = r["dot"]
        chosen = None
        if dot is not None and cands:
            near = [(self._dist_from_box(c[:4], dot["u"], dot["v"]), c) for c in cands]
            near = [t for t in near if t[0] <= DOT_TO_WIN_TOL_PX]
            if near:
                # 先看"这个词有多像目标"，再看得分，最后看红点离框边多近
                near.sort(key=lambda t: (-t[1][7], -t[1][5], t[0]))
                chosen = near[0][1]
                r["source"] = "dot+" + chosen[4]
        if chosen is None and dot is not None:
            d = dot["d_px"] or 1.0
            half = max(14.0, d * 1.6)
            chosen = [dot["u"] - half, dot["v"] - half, 2 * half, 2 * half, "dot", 1.0, "red dot", 1.0]
            r["source"] = "dot"
            r["note"] = "只见红点，未见窗户框"
        if chosen is None and cands:
            cands.sort(key=lambda c: (-c[7], -c[5],
                                      math.hypot(c[0] + c[2] / 2 - w / 2.0,
                                                 c[1] + c[3] / 2 - h / 2.0)))
            chosen = cands[0]
            r["source"] = chosen[4]
            r["note"] = "只见窗户，未见红点"
        if chosen is None:
            r["text"] = "看不清窗户，也不见红点 —— 请你手动挪一下再按 I"
            r["ms"]["total"] = round((time.time() - t_all) * 1000, 1)
            return r

        x, y, bw, bh, src, sc, lab = chosen[:7]
        cx, cy = x + bw / 2.0, y + bh / 2.0
        r["ok"] = True
        r["box"] = [round(x, 1), round(y, 1), round(x + bw, 1), round(y + bh, 1)]
        r["center"] = [round(cx, 1), round(cy, 1)]
        r["win"] = {"src": src, "score": sc, "label": lab,
                    "w_px": round(bw, 1), "h_px": round(bh, 1)}

        # 距离：优先红点（尺寸已知 0.20 m），否则按名义窗宽
        if dot is not None and dot["d_px"] > 2:
            d_true = dot["d_px"] * DOT_DIAM_K
            r["dist_m"] = round(FX * DOT_D_M / d_true, 1)
            r["dist_src"] = "dot_size"
            r["dist_err_pct"] = DIST_ERR_PCT
        elif src != "dot":
            r["dist_m"] = round(FX * WIN_W_M / max(bw, 1.0), 1)
            r["dist_src"] = "win_width"
            r["dist_err_pct"] = 35          # 窗宽假设误差更大
        r["height_m"] = self._height_m(cy, h, r["dist_m"])
        if r["height_m"] is not None:
            r["floor"] = "约 %d 楼" % max(1, int(round(r["height_m"] / 3.0)))

        # 一句话
        parts = []
        parts.append("窗户" if src != "dot" else "红点")
        if r["dist_m"]:
            parts.append("%.0f m" % r["dist_m"])
        if r["floor"]:
            parts.append(r["floor"])
        tag = []
        if dot is not None:
            tag.append("窗上有红点")
        if src != "dot":
            tag.append("%s 框" % src)
        r["text"] = " · ".join(parts) + ("（" + "、".join(tag) + "）" if tag else "")
        if r.get("note") or r.get("gate_note"):
            tags = [x for x in (r.get("note"), r.get("gate_note")) if x]
            # 用 ASCII 标记：msyh 里没有 ⚠ 字形，会显示成方块
            r["text"] += "  [!] " + "；".join(tags)
        r["ms"]["total"] = round((time.time() - t_all) * 1000, 1)
        return r

    def close(self):
        self.worker.close()

    # 让 with 语句能用
    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
        return False


# ══════════════════════════════════════════════════════════════════
# 画图（给网页/录屏/效果图共用）
# ══════════════════════════════════════════════════════════════════
def draw_result(bgr, r, scale=1.0, show_note=True):
    """把 find() 的结果画在图上：绿框=窗户，红圈=红点，底部一行字。"""
    vis = bgr.copy()
    h, w = vis.shape[:2]
    if r.get("box"):
        x1, y1, x2, y2 = [int(v) for v in r["box"]]
        col = (0, 255, 0) if r["source"] != "dot" else (0, 165, 255)
        cv2.rectangle(vis, (x1, y1), (x2, y2), col, max(2, int(2 * scale)))
        cv2.drawMarker(vis, (int(r["center"][0]), int(r["center"][1])), col,
                       cv2.MARKER_CROSS, max(10, int(14 * scale)), max(1, int(2 * scale)))
    if r.get("dot"):
        u, v = int(r["dot"]["u"]), int(r["dot"]["v"])
        cv2.circle(vis, (u, v), max(10, int(16 * scale)), (0, 0, 255), max(2, int(2 * scale)))
    if show_note and r.get("text"):
        bh = max(38, int(46 * scale))
        cv2.rectangle(vis, (0, h - bh), (w, h), (0, 0, 0), -1)
        fs = 0.62 * scale
        cv2.putText(vis, "[%s] %s" % (r["source"], r["text"]), (10, h - bh + int(bh * 0.68)),
                    cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 255, 255), max(1, int(2 * scale)))
        m = r.get("ms") or {}
        t = "dot %s  seg %s  gdino %s  total %s ms" % (
            m.get("dot", "-"), m.get("seg", "-"), m.get("gdino", "-"), m.get("total", "-"))
        cv2.putText(vis, t, (10, h - int(bh * 0.18)), cv2.FONT_HERSHEY_SIMPLEX,
                    fs * 0.7, (170, 170, 170), 1)
    return vis


if __name__ == "__main__":
    # 自测：python src/mcpv/window_finder.py <图片或视频帧编号>
    import argparse
    sys.path.insert(0, str(ROOT / "eval"))
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", type=int, default=254, help="录像帧号（用 bench_lat_crop 的 gt）")
    a = ap.parse_args()
    import bench_lat_crop as L
    fr = L.load_frames()
    bgr = fr[a.frame]["clean"]
    with WindowFinder() as wf:
        t0 = time.time()
        with wf.worker.lock:
            pass
        r = wf.find(bgr)
    print(json.dumps(r, ensure_ascii=False, indent=1))
    vis = draw_result(bgr, r)
    p = ROOT / "eval" / "figures" / ("wf_frame%d.jpg" % a.frame)
    ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if ok:
        Path(p).write_bytes(buf.tobytes())
    print("saved", p)
