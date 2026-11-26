# -*- coding: utf-8 -*-
"""vlm_advisor.py —— VLM 方向顾问（只画建议，不进飞控）

它做什么
──────────────────────────────────────────────────────────────
把 MiniCPM-V 从"复述 CV 的候选排序"改成"给粗方向的顾问"：

    CV 看不见目标时（远）  → 它说"目标在上/中/下 的 左/中/右"，屏幕画准星+箭头+水平参考线
    CV 刚看见时（1~2 个）  → 它复核"哪一格是白纸红圆"
    CV 稳住了（≥3 个）     → 它闭嘴，显示"CV 跟踪中"
    已经很近（≤5 m）       → 它看窗开/关、反光

**一切输出只到画面上，不写任何 ROS 话题、不发 ZMQ、不改选点。**

为什么这么问（都有实测依据，见 eval/vlm_strategy.json 等）
  · 横 3 带 / 竖 3 条各问一次：真实帧 78% / 80%，中值滤波后 87.5%
  · 问 4 带掉到 2/3、6 带掉到 1/3 —— 所以只用 3 选 1
  · 答错**只错 1 格**（40 帧零例外），1 格 = 12.7°，小于半视场 30.5°，飞不丢
  · 输入用 640 宽比 1280 **又快又准**（2.3 s vs 5.0 s；10/12 vs 8/12）
  · 编号必须**画进图里**，写在提示词里会 0/20

用法
    python scripts/vlm_advisor.py                 # 起 :8093
    网页「识别画面 [地址]」填  http://127.0.0.1:8093/snapshot.jpg
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import threading
import time
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"
FX = 1626.6183          # 1920 校正左目焦距
DOT_D = 0.20            # 红圆直径 (m)

Q_ROW = ("这张图被横向切成 3 条带编号的横带（编号画在每条带左上角）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号横带？只回答一个数字。")
Q_COL = ("这张图被竖向切成 3 条编号的竖条（编号画在每条竖条上端）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号竖条？只回答一个数字。")

# ---------- 中文字（cv2 不支持，用 PIL） ----------
_FONTS = {}


def _font(size):
    if size in _FONTS:
        return _FONTS[size]
    f = None
    for p in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
        if os.path.exists(p):
            try:
                from PIL import ImageFont
                f = ImageFont.truetype(p, size)
                break
            except Exception:
                pass
    _FONTS[size] = f
    return f


def put_cn(bgr, items):
    """items = [(text, (x,y), size, (B,G,R)), ...]，一次 PIL 往返画完。"""
    try:
        from PIL import Image, ImageDraw
    except Exception:
        for text, xy, size, col in items:
            cv2.putText(bgr, text, xy, cv2.FONT_HERSHEY_SIMPLEX,
                        size / 24.0, col, 2)
        return bgr
    img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(img)
    for text, xy, size, col in items:
        f = _font(size)
        if f is None:
            continue
        d.text(xy, text, font=f, fill=(col[2], col[1], col[0]))
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


# ---------- 问模型 ----------
def ask(img, prompt, npd=8):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        return ""
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(OLLAMA, data=body,
                                headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=120) as r:
        return (json.loads(r.read()).get("response") or "").strip()


def first_digit(txt, lo=1, hi=3):
    for ch in txt or "":
        if ch.isdigit() and lo <= int(ch) <= hi:
            return int(ch)
    return None


def draw_row_labels(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        y0, y1 = int(i * H / n), int((i + 1) * H / n)
        cv2.rectangle(out, (0, y0), (W - 1, y1), (0, 255, 255), 2)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (14, y0 + 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def draw_col_labels(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        x0, x1 = int(i * W / n), int((i + 1) * W / n)
        cv2.rectangle(out, (x0, 0), (x1 - 1, H - 1), (0, 255, 255), 2)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (x0 + 14, 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def median_of(seq):
    v = [x for x in seq if x is not None]
    return int(np.median(v)) if v else None


def tail_run(seq, val):
    """末尾连续等于 val 的个数（用于置信度 ●●●）。"""
    n = 0
    for x in reversed(list(seq)):
        if x == val:
            n += 1
        else:
            break
    return n


# ---------- HTTP 输出 ----------
class Holder(object):
    def __init__(self):
        self.jpeg = None
        self.info = {}
        self.lock = threading.Lock()
        self.last_ask = {"lock": False, "t": 0.0}

    def put(self, jpeg, info):
        with self.lock:
            self.jpeg, self.info = jpeg, info

    def get(self):
        with self.lock:
            return self.jpeg, dict(self.info)


def serve(port, holder):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/snapshot.jpg"):
                b, info = holder.get()
                if b is None:
                    self.send_response(503)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(b)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(b)
            elif self.path.startswith("/advice.json"):
                _, info = holder.get()
                out = json.dumps(info, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(out)))
                self.end_headers()
                self.wfile.write(out)
            else:
                msg = ("vlm_advisor 在跑。\n"
                       "  网页地址填: http://127.0.0.1:%d/snapshot.jpg\n"
                       "  调试信息:   http://127.0.0.1:%d/advice.json\n" % (port, port))
                b = msg.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)

    srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# ---------- 取数据 ----------
def get_json(url, timeout=6.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def get_bytes(url, timeout=6.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def fetch_cands(state_url, det_w, img_w):
    st = get_json(state_url)
    g = st.get("gate") or {}
    k = img_w / float(det_w)
    out = []
    for c in (g.get("cands") or []):
        uv = c.get("uv")
        if not uv or len(uv) < 2:
            continue
        out.append({"u": float(uv[0]) * k, "v": float(uv[1]) * k,
                    "ps": float(c.get("score") or 0.0),
                    "d_px": float(c.get("d_px") or 0.0) * k,
                    "d_raw": float(c.get("d_px") or 0.0)})
    out.sort(key=lambda q: -q["ps"])
    return out, g.get("stage"), bool(g.get("sim"))


# ---------- 主循环 ----------
def cycle(a, holder, hist_row, hist_col):
    try:
        raw = get_bytes(a.snap)
        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError("解不出图")
    except Exception as e:
        print("[顾问] 取图失败: %s" % e, flush=True)
        return

    if img.shape[1] > a.width:
        img = cv2.resize(img, (a.width, int(round(img.shape[0] * a.width / float(img.shape[1])))),
                         interpolation=cv2.INTER_AREA)
    H, W = img.shape[:2]

    try:
        cands, stage, sim = fetch_cands(a.state, a.det_width, W)
    except Exception as e:
        print("[顾问] 取状态失败: %s" % e, flush=True)
        cands, stage, sim = [], None, False

    # 距离用**检测分辨率**下的像素直径算：Z = f·D/d，f 与 d 必须同一尺度
    # （早期版本把 d 缩到预览图尺度却仍用 1920 的 f，距离会被放大约 3 倍）
    d_best = max([c["d_raw"] for c in cands], default=0.0)
    z = (FX * DOT_D / d_best) if d_best > 1.0 else None

    if not cands:
        mode = "SEEK"
    elif len(cands) <= 2:
        mode = "VERIFY"
    elif z is not None and z <= 5.0:
        mode = "SEMANTIC"
    else:
        mode = "TRACK"

    # --far-m: 远距一律给方向（远距 CV 候选多且杂，方向比编号有用；
    #          也是"大模型在远处起作用"最直观的呈现方式）
    if a.far_m and a.mode == "auto" and (z is None or z >= a.far_m):
        mode = "SEEK"

    # 两个纯模式：pick 旧行为（有候选就复核），advisor 新行为（有候选就不问）
    if a.mode == "pick" and cands:
        mode = "VERIFY"
    elif a.mode == "advisor" and mode == "VERIFY":
        mode = "TRACK"

    out = img.copy()
    info = {"t": time.strftime("%H:%M:%S"), "mode": mode, "n": len(cands),
            "z": round(z, 2) if z else None, "stage": stage, "sim": sim,
            "advice_only": True}

    def bar(lines, color=(0, 0, 0)):
        bh = max(46, H // 8)
        cv2.rectangle(out, (0, H - bh), (W, H), color, -1)
        items = []
        for i, (txt, sz, col) in enumerate(lines):
            items.append((txt, (8, H - bh + 4 + i * (sz + 3)), sz, col))
        return put_cn(out, items)

    if mode == "SEEK":
        # 远距时也可能已有候选（--far-m 强制走方向）；把候选淡淡画出来，
        # 让"CV 看到的"和"大模型建议的方向"同屏，不用来回切地址
        for i, c in enumerate(cands[:6], 1):
            cv2.circle(out, (int(c["u"]), int(c["v"])), 10, (120, 120, 120), 1)
            cv2.putText(out, str(i), (int(c["u"]) + 12, int(c["v"]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 120, 120), 1)
        row = col = None
        now = time.time()
        cached = holder.last_ask
        skip = (cached.get("lock") and now - cached.get("t", 0) < a.recheck)
        dt = 0.0
        if not a.no_vlm and not skip and a.mode in ("auto", "advisor"):
            t0 = time.time()
            try:
                row = first_digit(ask(draw_row_labels(img), Q_ROW))
                col = first_digit(ask(draw_col_labels(img), Q_COL))
            except Exception as e:
                print("[顾问] 问模型失败: %s" % e, flush=True)
            dt = time.time() - t0
            if row is not None:
                hist_row.append(row)
            if col is not None:
                hist_col.append(col)
            print("[顾问] SEEK 行=%s 列=%s  用时 %.1fs%s"
                  % (row, col, dt, "  (锁定中，跳过)" if skip else ""), flush=True)
        elif skip:
            print("[顾问] SEEK 锁定中，本轮跳过提问（%.0fs 后复问）" % a.recheck, flush=True)

        med_r, med_c = median_of(hist_row), median_of(hist_col)
        run_r, run_c = tail_run(hist_row, med_r), tail_run(hist_col, med_c)
        locked = (run_r >= 3 and run_c >= 3)
        info.update(row=med_r, col=med_c, run_r=run_r, run_c=run_c, lock=locked)
        holder.last_ask = {"row": med_r, "col": med_c, "lock": locked,
                           "t": now if dt else cached.get("t", 0)}

        if med_r and med_c:
            bh = max(46, H // 8)
            cy = int((med_r - 0.5) * H / 3.0)
            cx = int((med_c - 0.5) * W / 3.0)
            cy = max(26, min(H - bh - 26, cy))     # 别压在底部文字条上
            cx = max(26, min(W - 26, cx))
            colr = (0, 255, 0) if locked else (0, 165, 255)
            # 水平参考线（建议高度）
            for x in range(0, W, 24):
                cv2.line(out, (x, cy), (min(x + 12, W), cy), colr, 2)
            # 中心 → 建议点 的箭头
            cv2.arrowedLine(out, (W // 2, H // 2), (cx, cy), colr, 2, tipLength=0.12)
            cv2.circle(out, (cx, cy), 18, colr, 2)
            cv2.line(out, (cx - 26, cy), (cx + 26, cy), colr, 2)
            cv2.line(out, (cx, cy - 26), (cx, cy + 26), colr, 2)
            cv2.drawMarker(out, (W // 2, H // 2), (200, 200, 200),
                           cv2.MARKER_CROSS, 14, 1)
            out = put_cn(out, [("%d / %d" % (med_r, med_c),
                                (cx + 34, cy - 24), 14, colr)])
            rn = ["上", "中", "下"][med_r - 1]
            cn = ["左", "中", "右"][med_c - 1]
            lock_s = "●●●" if locked else ("●○○" if run_r or run_c else "○○○")
            out = bar([("VLM 建议：往【%s·%s】方向，先前进约 %.0f m  %s" % (rn, cn, a.step, lock_s), 17,
                        (46, 255, 46) if locked else (0, 210, 255)),
                       ("飞完这段等 CV 说话   %s" % info["t"], 13, (190, 190, 190))])
        else:
            out = bar([("VLM 正在找目标（第 %d/3 次）…" % (run_r + 1), 17, (0, 210, 255)),
                       ("%s" % info["t"], 13, (190, 190, 190))])

    elif mode == "VERIFY":
        pick = None
        if a.mode in ("auto", "pick") and not a.no_vlm:
            try:
                from mcpv.pick_core import suggest
                use = [{"cx": c["u"], "cy": c["v"], "d": max(6.0, c["d_px"])}
                       for c in cands[:a.pick_max]]
                t0 = time.time()
                res = suggest(img, use, timeout=40)
                pick = res.get("pick")
                print("[顾问] VERIFY 候选 %d 个 → VLM 选 #%s  用时 %.1fs"
                      % (len(use), pick, time.time() - t0), flush=True)
            except Exception as e:
                print("[顾问] 复核失败: %s" % e, flush=True)
        info["pick"] = pick
        for i, c in enumerate(cands[:6], 1):
            col = (0, 255, 0) if pick == i else (0, 165, 255)
            r = int(max(12, min(60, c["d_px"] * 1.1)))
            cv2.circle(out, (int(c["u"]), int(c["v"])), r, col, 2)
            cv2.putText(out, str(i), (int(c["u"]) + r + 3, int(c["v"]) - r),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(out, str(i), (int(c["u"]) + r + 3, int(c["v"]) - r),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
        head = ("VLM 复核：建议第 %d 个" % pick) if pick else \
               ("VLM 复核：CV 有 %d 个候选（未给出）" % len(cands))
        out = bar([(head, 17,
                    (46, 255, 46) if pick else (0, 210, 255)),
                   ("距离估计 %s   %s" % (("%.1f m" % z) if z else "未知", info["t"]), 13, (190, 190, 190))])

    elif mode == "TRACK":
        for i, c in enumerate(cands[:6], 1):
            cv2.circle(out, (int(c["u"]), int(c["v"])), 14, (0, 255, 0), 2)
        out = bar([("CV 跟踪中（%d 个候选）" % len(cands), 17, (46, 255, 46)),
                   ("距离约 %s   %s" % (("%.1f m" % z) if z else "未知", info["t"]), 13, (190, 190, 190))])

    else:   # SEMANTIC
        out = bar([("已经 %s m：VLM 看窗况（开/关、反光）" % (("%.1f" % z) if z else "?"), 17, (46, 255, 46)),
                   ("%s" % info["t"], 13, (190, 190, 190))])
        info["semantic_pending"] = True

    ok, buf = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    if ok:
        holder.put(buf.tobytes(), info)
        if a.save_dir:
            Path(a.save_dir).mkdir(parents=True, exist_ok=True)
            p = Path(a.save_dir) / ("adv_%s.jpg" % time.strftime("%H%M%S"))
            cv2.imwrite(str(p), out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default="http://127.0.0.1:8080/api/state")
    ap.add_argument("--snap", default="http://127.0.0.1:8091/snapshot.jpg")
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--det-width", type=int, default=1920, help="cands 的 uv 是哪个分辨率")
    ap.add_argument("--width", type=int, default=640, help="送给模型的图宽（640 实测最快最准）")
    ap.add_argument("--interval", type=float, default=3.0)
    ap.add_argument("--step", type=float, default=4.0, help="建议前进步长 (m)")
    ap.add_argument("--recheck", type=float, default=10.0,
                    help="已锁定时隔多久才复问一次 (s)，省算力")
    ap.add_argument("--mode", choices=["auto", "advisor", "pick"], default="auto",
                    help="auto=按 CV 状态自动切换；advisor=只给方向；pick=只做选编号")
    ap.add_argument("--pick-max", type=int, default=6,
                    help="复核时最多看几个候选")
    ap.add_argument("--far-m", type=float, default=0.0,
                    help="距离 ≥ 这个米数时一律给'方向'建议（0=关）。"
                         "远距时 CV 的候选往往又多又杂，'往哪飞'比'选哪个'更有用")
    ap.add_argument("--once", type=int, default=0, help="跑 N 轮就退出（测试用）")
    ap.add_argument("--no-vlm", action="store_true", help="不问模型，只测界面")
    ap.add_argument("--save-dir", default=None)
    a = ap.parse_args()

    holder = Holder()
    serve(a.port, holder)
    print("VLM 顾问已起:  http://127.0.0.1:%d/snapshot.jpg" % a.port, flush=True)
    print("               http://127.0.0.1:%d/advice.json" % a.port, flush=True)
    print("  源: %s\n      %s" % (a.state, a.snap), flush=True)
    print("  只出建议，不写任何 ROS 话题 / ZMQ 包，不影响飞行", flush=True)

    hist_row, hist_col = deque(maxlen=5), deque(maxlen=5)
    n = 0
    while True:
        cycle(a, holder, hist_row, hist_col)
        n += 1
        if a.once and n >= a.once:
            break
        time.sleep(a.interval)
    _, info = holder.get()
    print("最后一轮: %s" % json.dumps(info, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
