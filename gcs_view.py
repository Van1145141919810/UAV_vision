#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gcs_view.py — 地面站: 看 / 录 飞机**机上检测**的结果画面 (配飞机 vision_bridge _mode:=gate)
===============================================================================
2026-11-05。飞机桥在 gate 模式 (机上检测, 红点现在只能这样跑) 下, 每帧把
    校正后左目 + 检测叠加 (机上 Detector.draw 画好的) 的 JPEG  +  这一帧完整的 /gate/status
发到 tcp://<飞机>:5555。gcs_gate.py 只处理 gate_gcs 的并排帧, 收到这种帧只会告警 —— 这里专门接它。

本程序**不做检测**, 不需要标定 / vb09.perception, 只依赖 OpenCV + numpy + msgpack + pyzmq
(Windows / Linux 都行)。目标类型 (环 / 红点) 由飞机决定, 这里按 /gate/status 里有什么字段就显示什么。

用法
-------------------------------------------------------------------------------
  飞机:  python3 run_vision.py --target reddot ... (= vision_bridge _mode:=gate _target:=reddot)
  地面:  python gcs_view.py --drone 192.168.2.20                       看
         python gcs_view.py --drone 192.168.2.20 --record rec_view/    看 + 录 (启动就开始录)
         python gcs_view.py --drone 192.168.2.20 --record rec_view/ --rec-paused   按 R 才开始录
         python gcs_view.py --drone 192.168.2.20 --headless --record rec_view/     不开窗口只录
  没飞机时自测: python tools/fake_uplink.py <reddot_exp 录像目录>  (另一个终端) +
               python gcs_view.py --drone 127.0.0.1

窗口按键:  Q / Esc 退出   R 开始 / 暂停录像   S 存一张截图 (PNG, 带 HUD)   空格 冻结画面 (照收照录)

录像 (--record DIR, 每次启动一个子目录 view_YYYYmmdd_HHMMSS/)
-------------------------------------------------------------------------------
  video.mp4      看到的画面 (默认带 HUD; --rec-clean = 只录飞机发来的检测图)。
                 按**拍摄时刻 t_cap** 排帧 (固定 --rec-fps), 帧间空档用上一帧补齐, 回放速度 = 真实速度;
                 断链 > --rec-max-gap 秒不补, 只在 frames.jsonl 记一笔。mp4v 打不开就退回 MJPG .avi
  frames.jsonl   每收到一帧一行: seq / t_cap / t_recv / 视频帧号 vidx / /gate/status 原样 / 位姿 / 飞控 / 阶段
                 —— 回放时按 vidx 对上视频
  shot_*.png     S 键截图
  info.json      启动参数、飞机地址、首帧尺寸

下行 (回包)
-------------------------------------------------------------------------------
  默认在 tcp://*:5556 回 {t_cap 原样, found=false, sim=true, mode=0} —— 只为让飞机统计往返时延
  (桥在 gate 模式下忽略地面回包里的目标, 见 vision_bridge downlink_loop)。--no-reply 不回。
  和 gcs_gate.py / gcs_aruco_det2.py 用同一组端口, 三选一运行。

网页「识别画面」
-------------------------------------------------------------------------------
  同 gcs_gate: http://127.0.0.1:8091/snapshot.jpg (另一台电脑看加 --preview-host 0.0.0.0)。
  X-Det 就是飞机的 /gate/status 原样 + 链路统计 (fps / 往返 / 上行大小), 网页 cam.js 直接认。
===============================================================================
"""
import argparse
import json
import math
import os
import re
import socket
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from vb09.common.wire import pack, unpack, require_msgpack, zmq   # noqa: E402

require_msgpack("必须安装 msgpack: pip install msgpack")

UP_PORT, DOWN_PORT = 5555, 5556
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, YELLOW, RED, GRAY, WHITE, CYAN = (80, 230, 80), (0, 210, 255), (60, 60, 255), (170, 170, 170), \
    (240, 240, 240), (255, 220, 120)


# =============================================================================
# 收图: 专职线程读空 socket, 只留最新一帧 (同 gcs_gate 的 FrameSlot)
# =============================================================================
class FrameSlot(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._raw = None
        self._ev = threading.Event()
        self.n_recv = 0
        self.n_skip = 0

    def put(self, raw):
        with self._lock:
            if self._raw is not None:
                self.n_skip += 1
            self._raw = raw
            self.n_recv += 1
        self._ev.set()

    def get(self, timeout=0.5):
        if not self._ev.wait(timeout):
            return None
        with self._lock:
            raw, self._raw = self._raw, None
            self._ev.clear()
        return raw


def reader_loop(sub, slot, stop):
    while not stop.is_set():
        try:
            if not sub.poll(200):
                continue
            slot.put(sub.recv())
            while True:
                try:
                    slot.put(sub.recv(zmq.NOBLOCK))
                except zmq.Again:
                    break
        except Exception as e:                      # noqa: BLE001
            print("[gcs_view] 收图线程异常: %s" % e)
            time.sleep(0.2)


# =============================================================================
# 链路统计
# =============================================================================
class Link(object):
    def __init__(self):
        self.n = 0
        self.t_last = None
        self.dt = deque(maxlen=30)
        self.kb = deque(maxlen=30)
        self.last_seq = None
        self.lost = 0
        self.rtt = None
        self.rtt_p95 = None
        self.stall = 0

    def tick(self, head, nbytes):
        now = time.time()
        if self.t_last is not None:
            self.dt.append(now - self.t_last)
        self.t_last = now
        self.kb.append(nbytes / 1024.0)
        self.n += 1
        seq = int(head.get("seq") or 0)
        if self.last_seq is not None and seq > self.last_seq + 1:
            self.lost += seq - self.last_seq - 1
        if seq:
            self.last_seq = seq
        if head.get("rtt_n"):
            self.rtt, self.rtt_p95, self.stall = head.get("rtt_p50"), head.get("rtt_p95"), head.get("rtt_stall") or 0

    def fps(self):
        return (len(self.dt) / sum(self.dt)) if self.dt and sum(self.dt) > 0 else 0.0

    def loss_pct(self):
        tot = self.n + self.lost
        return 100.0 * self.lost / tot if tot else 0.0

    def kb_avg(self):
        return sum(self.kb) / len(self.kb) if self.kb else 0.0


# =============================================================================
# HUD (英文: OpenCV 自带字体画不了中文)
# =============================================================================
# 红点 locate.normal_consensus 的 n_why (中文) → 英文
_WHY = [
    (r"^一致的视角桶 (\d+) < (\d+)", "consistent views {0} < {1}"),
    (r"^视角桶 (\d+) < (\d+)", "views {0} < {1} (hover = 1 view)"),
    (r"^视角跨度 ([\d.]+)° < (\d+)°", "view span {0} < {1} deg (need up/down scan)"),
    (r"^两个解都说得通 \(代价差 ([-\d.inf]+) < (\d+)\)", "ambiguous: cost gap {0} < {1}"),
    (r"^近正对 \(倾角 ≈ (\d+)°\)", "near-frontal (tilt ~{0} deg): n ~ line of sight"),
]
_STAGE_COL = {"far": GREEN, "mid": GREEN, "near": GREEN, "terminal": GREEN, "coast": YELLOW,
              "blind": YELLOW, "lost": RED, "search": GRAY}


def ascii_txt(s):
    """OpenCV 只画 ASCII: 常见符号换掉, 其余非 ASCII 字符 → '?'。"""
    s = str(s).replace("×", "x").replace("°", "deg").replace("±", "+-").replace("σ", "sd").replace("≈", "~")
    return "".join(c if ord(c) < 128 else "?" for c in s)


def why_en(s):
    for pat, fmt in _WHY:
        m = re.match(pat, s or "")
        if m:
            return fmt.format(*m.groups())
    return ascii_txt(s)


def _f(v, fmt="%.2f", none="--"):
    try:
        if v is None or not math.isfinite(float(v)):
            return none
        return fmt % float(v)
    except (TypeError, ValueError):
        return none


def _vec(v, fmt="%+.2f"):
    return "(" + ",".join(_f(x, fmt) for x in v[:3]) + ")" if isinstance(v, (list, tuple)) and len(v) >= 3 else "--"


def hud_lines(head, st, link, rec_txt, gap_txt):
    """→ [(文字, 颜色)]。st = 飞机的 /gate/status (可能为 None)。"""
    L = []
    fc = head.get("fc") or {}
    ph = {}
    if head.get("ph"):
        try:
            ph = json.loads(head["ph"])
        except Exception:                           # noqa: BLE001
            ph = {}
    st = st or {}
    kind = st.get("kind") or ("ring" if "ring_w" in st or "ring_c" in st else "?")
    stage = st.get("stage") or "--"
    pub = st.get("published")
    tag = "PUBLISHED" if pub else ("held (not published)" if st.get("target") else "no target")
    L.append(("%s  stage %s  %s%s" % (kind.upper(), stage, tag, "  [SIM]" if st.get("sim") else ""),
              _STAGE_COL.get(stage, WHITE) if pub else (YELLOW if st.get("target") else RED)))
    phase = ph.get("phase", "-")
    sub = ph.get("sub_mode") or ""
    L.append(("FC %s %s   phase %s%s%s" % (fc.get("mode", "?"), "ARMED" if fc.get("armed") else "disarmed",
                                           phase, ("/" + sub) if sub else "",
                                           "   ABORT" if ph.get("abort") else ("   LAND" if ph.get("land") else "")),
              RED if ph.get("abort") else (YELLOW if fc.get("armed") else GRAY)))
    if st.get("target"):
        sd = st.get("sd_cm") or []
        L.append(("target %s  dist %s m  src %s  sd %s cm  age %s ms  pose %s"
                  % (_vec(st["target"]), _f(st.get("dist")), st.get("src", "--"),
                     "/".join(_f(x, "%.1f") for x in sd) if sd else "--", _f(st.get("age_ms"), "%.0f"),
                     st.get("pose", "?")), WHITE))
    else:
        L.append(("target --   pose %s   rejected: %s" % (st.get("pose", "?"), ascii_txt(" ".join(st.get("rejected") or [])) or "-"),
                  GRAY))
    if kind == "reddot" or "trk_id" in st:
        d = st.get("diag") or {}
        L.append(("dot  trk %s  hits %s  score %s  D %s cm (eff %s)  Z %s (st %s / size %s)  cands %s"
                  % (st.get("trk_id", "-"), st.get("n_hit", "-"), _f(st.get("score")),
                     _f((st.get("D_est") or float("nan")) * 100, "%.1f"),
                     _f((st.get("D_eff") or float("nan")) * 100, "%.1f"), _f(d.get("z")),
                     _f(d.get("z_st") if (d.get("z_st") or -1) > 0 else None), _f(d.get("z_sz")),
                     st.get("n_cand", len(st.get("cands") or []))), CYAN))
        if st.get("n"):
            L.append(("normal %s  +-%s deg  span %s deg  views %s"
                      % (_vec(st["n"], "%+.3f"), _f(st.get("n_sd_deg"), "%.1f"), _f(st.get("n_span_deg"), "%.0f"),
                         st.get("n_k", "-")), GREEN if (st.get("n_sd_deg") or 99) <= 3.0 else YELLOW))
        else:
            L.append(("normal --  %s  (bins %s)" % (why_en(st.get("n_why") or "no data"), st.get("n_vbins", st.get("n_bins", 0))), YELLOW))
    else:
        rg = st.get("ring") or {}
        L.append(("ring %s  src %s  corners %s  n %s  aim %s"
                  % ("ok" if rg.get("ok") else "--", rg.get("src") or "-", rg.get("corners", "-"),
                     _vec(st.get("n"), "%+.3f"), _vec(st.get("aim"))), CYAN))
    det_ms = st.get("ms")
    L.append(("link %.1f fps  %.0f KB  loss %.1f%%  rtt p50 %s / p95 %s ms  stall %s  det %s ms  %dx%d%s"
              % (link.fps(), link.kb_avg(), link.loss_pct(), _f(link.rtt, "%.0f"), _f(link.rtt_p95, "%.0f"),
                 link.stall, _f(det_ms, "%.0f"), int(head.get("w") or 0), int(head.get("h") or 0),
                 ("   " + gap_txt) if gap_txt else ""), RED if gap_txt else WHITE))
    if rec_txt:
        L.append((rec_txt, RED))
    return L


LINE_H = 30


def compose(img, lines, view_w):
    """检测图缩到 view_w 宽, 上面拼一条黑底 HUD (不盖住画面)。"""
    h, w = img.shape[:2]
    if view_w and w != view_w:
        img = cv2.resize(img, (view_w, int(round(h * view_w / float(w)))),
                         interpolation=cv2.INTER_LINEAR if view_w > w else cv2.INTER_AREA)
    W = img.shape[1]
    fs = max(0.45, min(0.75, W / 1920.0 * 0.75))
    lh = int(LINE_H * fs / 0.75)
    band = np.zeros((lh * len(lines) + 10, W, 3), np.uint8)
    for i, (txt, col) in enumerate(lines):
        cv2.putText(band, ascii_txt(txt), (8, lh * (i + 1)), FONT, fs, col, 1 if fs < 0.6 else 2, cv2.LINE_AA)
    return np.vstack([band, img])


# =============================================================================
# 录像
# =============================================================================
class Recorder(object):
    def __init__(self, root, fps, max_gap, clean, args_dict):
        self.dir = os.path.join(root, time.strftime("view_%Y%m%d_%H%M%S"))
        os.makedirs(self.dir, exist_ok=True)
        self.fps, self.max_gap, self.clean = float(fps), float(max_gap), clean
        self.vw = None
        self.size = None
        self.path = None
        self.t0 = None          # t_cap 对应视频第 0 帧
        self.vidx = -1          # 最后写入的视频帧号
        self.last = None
        self.n_frames = 0
        self.jf = open(os.path.join(self.dir, "frames.jsonl"), "w", encoding="utf-8")
        self.info = {"args": args_dict, "start": time.strftime("%Y-%m-%d %H:%M:%S")}
        self.paused = False
        self.t_rec = 0.0        # 实际录了多少秒 (视频时长)

    def _open(self, img):
        h, w = img.shape[:2]
        self.size = (w, h)
        for ext, cc in ((".mp4", "mp4v"), (".avi", "MJPG")):
            p = os.path.join(self.dir, "video" + ext)
            vw = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*cc), self.fps, self.size)
            if vw.isOpened():
                self.vw, self.path = vw, p
                break
            vw.release()
        if self.vw is None:
            raise RuntimeError("VideoWriter 打不开 (mp4v / MJPG 都不行)")
        self.info.update(video=os.path.basename(self.path), size=[w, h], fps=self.fps)
        self._write_info()
        print("[gcs_view] 开始录像: %s  %dx%d @%.0ffps" % (self.path, w, h, self.fps))

    def _write_info(self):
        with open(os.path.join(self.dir, "info.json"), "w", encoding="utf-8") as fh:
            json.dump(self.info, fh, ensure_ascii=False, indent=1)

    def add(self, img, t_cap, rec_row):
        """img = 要录的画面; t_cap = 拍摄时刻 (飞机时钟)。→ (这一帧的视频帧号 或 None, 空档秒数 或 None)"""
        gap = None
        if self.paused:
            rec_row["vidx"] = None
            self.jf.write(json.dumps(rec_row, ensure_ascii=False, default=str) + "\n")
            return None, None
        if self.vw is None:
            self._open(img)
        if img.shape[1] != self.size[0] or img.shape[0] != self.size[1]:
            img = cv2.resize(img, self.size, interpolation=cv2.INTER_AREA)
        if self.t0 is None:
            self.t0 = t_cap
        idx = int(round((t_cap - self.t0) * self.fps))
        if idx <= self.vidx:                         # 比录像帧率还快 / 时间倒退: 不另起一帧, 只记数据
            rec_row["vidx"] = self.vidx
        else:
            n_fill = idx - self.vidx - 1
            if n_fill > self.max_gap * self.fps:     # 断链太久: 不补帧, 时间轴往前挪
                gap = n_fill / self.fps
                self.t0 += (n_fill - 1) / self.fps
                idx = self.vidx + 1
                n_fill = 0
                self.jf.write(json.dumps({"gap_s": round(gap, 2), "t_cap": t_cap}) + "\n")
            for _ in range(n_fill):
                self.vw.write(self.last)
            self.vw.write(img)
            self.vidx = idx
            self.last = img
            rec_row["vidx"] = idx
        self.n_frames += 1
        self.t_rec = (self.vidx + 1) / self.fps
        self.jf.write(json.dumps(rec_row, ensure_ascii=False, default=str) + "\n")
        return rec_row["vidx"], gap

    def toggle(self):
        self.paused = not self.paused
        if not self.paused and self.vidx >= 0:
            self.t0 = None                            # 继续录: 新的时间基准接在已录的后面
            self._resume = True
        print("[gcs_view] 录像%s" % ("暂停" if self.paused else "继续"))

    def fix_resume(self, t_cap):
        if getattr(self, "_resume", False):
            self.t0 = t_cap - (self.vidx + 1) / self.fps
            self._resume = False

    def snapshot(self, img):
        p = os.path.join(self.dir, time.strftime("shot_%H%M%S.png"))
        cv2.imwrite(p, img)
        return p

    def close(self):
        if self.vw is not None:
            self.vw.release()
        self.info.update(frames_received=self.n_frames, video_frames=self.vidx + 1,
                         video_seconds=round(self.t_rec, 2), end=time.strftime("%Y-%m-%d %H:%M:%S"))
        self._write_info()
        self.jf.close()
        print("[gcs_view] 录像保存在 %s: 收到 %d 帧, 视频 %d 帧 / %.1f s"
              % (self.dir, self.n_frames, self.vidx + 1, self.t_rec))


# =============================================================================
# 杂项 (同 gcs_gate)
# =============================================================================
def _dpi_aware():
    if os.name != "nt":
        return
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:                           # noqa: BLE001
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:                               # noqa: BLE001
        pass


def _lan_ips(drone):
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((drone or "192.168.2.20", 9))
        ips.add(s.getsockname()[0])
        s.close()
    except Exception:                               # noqa: BLE001
        pass
    return sorted(i for i in ips if not i.startswith("127."))


# =============================================================================
# main
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description="看 / 录 飞机机上检测 (vision_bridge _mode:=gate) 的结果画面")
    ap.add_argument("--drone", required=True, help="飞机 IP (自测: 127.0.0.1 + tools/fake_uplink.py)")
    ap.add_argument("--record", default=None, help="录像根目录 (每次启动建一个 view_日期_时间/ 子目录)")
    ap.add_argument("--rec-paused", action="store_true", help="启动时先不录, 按 R 开始")
    ap.add_argument("--rec-fps", type=float, default=10.0, help="录像帧率 (按拍摄时刻排帧), 默认 10 = 桥 gate 模式默认 tx_hz")
    ap.add_argument("--rec-max-gap", type=float, default=2.0, help="断链超过这么多秒不补帧, 默认 2")
    ap.add_argument("--rec-clean", action="store_true", help="录像不带 HUD, 只录飞机发来的检测图")
    ap.add_argument("--view-w", type=int, default=1280, help="窗口 / 录像画面宽度 (检测图 960 宽, 放大显示), 0 = 原尺寸")
    ap.add_argument("--headless", action="store_true", help="不开本地窗口 (只录 / 只给网页)")
    ap.add_argument("--no-reply", action="store_true", help="不回包 (飞机就统计不了往返时延)")
    ap.add_argument("--no-preview", action="store_true", help="不开网页预览服务")
    ap.add_argument("--preview-host", default="127.0.0.1", help="另一台电脑也要看时用 0.0.0.0")
    ap.add_argument("--preview-port", type=int, default=8091)
    ap.add_argument("--preview-width", type=int, default=960)
    ap.add_argument("--max-seconds", type=float, default=0.0, help=argparse.SUPPRESS)   # 自测用: 跑这么久自动退出
    args = ap.parse_args()
    if zmq is None:
        sys.exit("pip install pyzmq")

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 4)
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.connect("tcp://%s:%d" % (args.drone, UP_PORT))
    pub = None
    if not args.no_reply:
        pub = ctx.socket(zmq.PUB)
        pub.setsockopt(zmq.SNDHWM, 4)
        pub.setsockopt(zmq.LINGER, 0)
        try:
            pub.bind("tcp://*:%d" % DOWN_PORT)
        except zmq.ZMQError as e:
            print("[gcs_view] ⚠ 下行端口 %d 绑不上 (%s) —— gcs_gate / gcs_aruco_det2 还开着? 先关掉它们。"
                  "这次不回包 (飞机统计不了往返)" % (DOWN_PORT, e))
            pub = None
    slot, stop = FrameSlot(), threading.Event()
    threading.Thread(target=reader_loop, args=(sub, slot, stop), daemon=True).start()
    print("[gcs_view] 上行 SUB tcp://%s:%d   下行 %s" % (args.drone, UP_PORT,
                                                     "PUB tcp://*:%d (只为往返统计)" % DOWN_PORT if pub else "不回包"))

    pv = None
    preview_server = None
    if not args.no_preview:
        try:
            from vb09.common import preview_server
            pv = preview_server.start(port=args.preview_port, host=args.preview_host, max_width=args.preview_width)
        except ImportError:
            print("[gcs_view] 没找到 vb09/common/preview_server.py —— 网页看不到画面, 本地照常")
    if pv is not None:
        print("[gcs_view] 网页「识别画面 → 地址」: 本机留空 (= http://127.0.0.1:%d/snapshot.jpg)" % args.preview_port)
        if args.preview_host in ("0.0.0.0", ""):
            for ip in _lan_ips(args.drone) or ["<本机IP>"]:
                print("        另一台电脑: http://%s:%d/snapshot.jpg" % (ip, args.preview_port))

    rec = None
    if args.record:
        rec = Recorder(args.record, args.rec_fps, args.rec_max_gap, args.rec_clean, vars(args))
        rec.paused = args.rec_paused
        print("[gcs_view] 录像目录 %s%s" % (rec.dir, "  (暂停中, 按 R 开始)" if rec.paused else ""))

    win = "gcs_view"
    if not args.headless:
        _dpi_aware()
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    link = Link()
    warned = set()
    frozen = None
    gap_until, gap_txt = 0.0, ""
    t_start = t_print = time.time()
    last_show = None
    try:
        while not stop.is_set():
            if args.max_seconds and time.time() - t_start > args.max_seconds:
                break
            raw = slot.get(timeout=0.3)
            key = (cv2.waitKey(1) & 0xFF) if not args.headless else 255
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("r"), ord("R")):
                if rec is None:
                    rec = Recorder(os.path.join(HERE, "rec_view"), args.rec_fps, args.rec_max_gap,
                                   args.rec_clean, vars(args))
                    print("[gcs_view] 没给 --record, 录到 %s" % rec.dir)
                else:
                    rec.toggle()
            if key in (ord("s"), ord("S")) and last_show is not None:
                if rec is None:
                    p = os.path.join(HERE, time.strftime("shot_%H%M%S.png"))
                    cv2.imwrite(p, last_show)
                else:
                    p = rec.snapshot(last_show)
                print("[gcs_view] 截图 %s" % p)
            if key == ord(" "):
                frozen = None if frozen is not None else last_show
                print("[gcs_view] 画面%s (照收照录)" % ("冻结" if frozen is not None else "继续"))
            if raw is None:
                if "noframe" not in warned and time.time() - t_start > 5.0 and link.n == 0:
                    print("[gcs_view] 5s 没收到任何帧 —— 飞机上 run_vision.py (gate 模式) 起了吗? 地址对吗?")
                    warned.add("noframe")
                continue
            try:
                head = unpack(raw)
                jpg = head["jpg"]
            except Exception as e:                  # noqa: BLE001
                print("[gcs_view] 解包失败: %s" % e)
                continue
            link.tick(head, len(raw))
            t_cap = float(head.get("t_cap") or time.time())
            if pub is not None:
                try:
                    pub.send(pack({"seq": head.get("seq", 0), "t_cap": head.get("t_cap"), "t_done": time.time(),
                                   "found": False, "target": None, "sim": True, "mode": 0, "model": "view"}),
                             flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass
            src = head.get("src")
            if src != "gate_left_rect" and "src" not in warned:
                print("[gcs_view] ★ 收到的是 src=%s —— 不是机上检测画面 (飞机桥要 _mode:=gate)。照样显示, 但没有检测数据"
                      % src)
                warned.add("src")
            st = head.get("gate") if isinstance(head.get("gate"), dict) else None
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            if head.get("sbs"):                     # gate_gcs 的并排原图: 只显示左半
                img = img[:, : img.shape[1] // 2]

            if rec is not None and not rec.paused:
                rec.fix_resume(t_cap)
            rec_txt = ""
            if rec is not None:
                rec_txt = ("REC paused  %s" % os.path.basename(rec.dir)) if rec.paused else \
                    ("REC %02d:%02d  %s" % (int(rec.t_rec) // 60, int(rec.t_rec) % 60, os.path.basename(rec.dir)))
            if time.time() > gap_until:
                gap_txt = ""
            lines = hud_lines(head, st, link, rec_txt, gap_txt)
            show = compose(img, lines, args.view_w)
            last_show = show
            if rec is not None:
                row = {"seq": head.get("seq"), "t_cap": t_cap, "t_recv": time.time(), "w": head.get("w"),
                       "h": head.get("h"), "kb": round(len(jpg) / 1024.0, 1), "status": st,
                       "pose": head.get("pose"), "fc": head.get("fc"), "ph": head.get("ph"),
                       "rtt_p50": head.get("rtt_p50")}
                frame = img if args.rec_clean else show
                if args.rec_clean and args.view_w and img.shape[1] != args.view_w:
                    frame = cv2.resize(img, (args.view_w, int(round(img.shape[0] * args.view_w / float(img.shape[1])))))
                _, gap = rec.add(frame, t_cap, row)
                if gap:
                    gap_txt, gap_until = "link gap %.1fs" % gap, time.time() + 3.0
                    print("[gcs_view] 断链 %.1f s (录像里没有补帧)" % gap)
            if pv is not None:
                meta = dict(st or {})
                meta.update(kind="gate", fps=link.fps(), rtt=link.rtt, stall=link.stall,
                            up_w=head.get("w"), up_h=head.get("h"), up_kb=len(jpg) / 1024.0,
                            found=bool((st or {}).get("target")),
                            reason="" if (st or {}).get("target") else ((st or {}).get("stage") or "没有状态"))
                try:
                    preview_server.publish(img, meta=meta, raw=jpg)
                except Exception as e:              # noqa: BLE001
                    if "pv" not in warned:
                        print("[gcs_view] ⚠ 网页预览出错 (忽略): %s" % e)
                        warned.add("pv")
            if not args.headless:
                cv2.imshow(win, frozen if frozen is not None else show)
            if time.time() - t_print > 2.0:
                t_print = time.time()
                s = st or {}
                print("[gcs_view] %.1f fps  丢 %.1f%%  往返 p50 %s ms  阶段 %s  距离 %s  %s%s"
                      % (link.fps(), link.loss_pct(), _f(link.rtt, "%.0f"), s.get("stage", "--"), _f(s.get("dist")),
                         ("n ±%s°" % _f(s.get("n_sd_deg"), "%.1f")) if s.get("n") else
                         ("无法向: %s" % s.get("n_why")) if s.get("n_why") else "",
                         ("  | " + rec_txt) if rec_txt else ""))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if rec is not None:
            rec.close()
        if not args.headless:
            cv2.destroyAllWindows()
        if pv is not None:
            try:
                preview_server.stop()
            except Exception:                       # noqa: BLE001
                pass
        print("[gcs_view] 收工: 收到 %d 帧, 丢 %.1f%%" % (link.n, link.loss_pct()))


if __name__ == "__main__":
    main()
