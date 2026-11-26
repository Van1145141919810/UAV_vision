# -*- coding: utf-8 -*-
"""breach_run.py —— 真机版：红点跟踪 → 接近 → 交接（破窗前置）

═══════════════════════════════════════════════════════════════════
和 demo_fly.py 的区别
───────────────────────────────────────────────────────────────────
demo_fly.py  是"拍一帧 → 点选 → 发一步位移"的**点击式**demo。
本脚本是**连续跟踪**：一个窗口、一个画面源、一路跟到底，不再切模式。

═══════════════════════════════════════════════════════════════════
流程
───────────────────────────────────────────────────────────────────
  ACQUIRE  CV 扫红色候选（可叠加一次 VLM 语义指认做排序）
     ↓  人：单击选中 → 回车确认
  TRACK    模板 NCC 跟踪器初始化（含周围纹理）
     ↓
  APPROACH 每帧：模板匹配 + 背景光流校正 → 算 d_px → 算距离
           按刹车预测调速 → 发速度指令给飞机
     ↓  d_px 达到交接阈值
  HANDOFF  停住、输出交接 JSON（给文豪）

  异常：丢失 → 光流外推继续飞（减速），找回来就继续
        人按 WASD 立即接管；松手按 T 恢复
        距离 < 硬底线 → 强制刹车并轻退（防撞墙）

═══════════════════════════════════════════════════════════════════
画面源（只有一种，不再切模式）
───────────────────────────────────────────────────────────────────
优先 TCP vstream（机上 vstream.py 推，~10 fps）；
连不上时退回 scp 拉单帧（1.5 s/帧，很慢，只作兜底）。
启动时会把**实际分辨率**打印出来——这是我们最需要知道的数。
"""
import argparse
import json
import math
import socket
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402
from mcpv.tracker_core import TemplateTracker              # noqa: E402

# ═══════════════════════════════════════════════════════════════════
#  可调参数（集中在这里；后续调参只改这一块）
# ═══════════════════════════════════════════════════════════════════

# ── ① 飞机与通信 ─────────────────────────────────────────────
DRONE     = "visbot@192.168.2.20"
DRONE_IP  = "192.168.2.20"
PORT      = 8850            # UDP 控制
VIDEO_PORT = 8890           # TCP 视频流
SSH_KEY   = str(Path.home() / ".ssh" / "dsh_drone")

# ── ② 相机与目标 ────────────────────────────────────────────
# ★ 相机内参（直接取自同学 S14 的 stereo_calib.yaml，2026-11-04 标定，
#   41 对图，rms_left=0.77 px，左目 1920x1080）。
#   注意：主点**不是**画面中心！cx=932.3 而 W/2=960，差 27.7 px；
#   cy=517.3 而 H/2=540，差 22.7 px。用 W/2 当主点会让"正中"判断偏 1°。
FX_PX_1920 = 1566.149       # fx
FY_PX_1920 = 1573.382       # fy（与 fx 差 0.46%，垂直控制要用 fy）
CX_PX_1920 = 932.277        # cx（主点横坐标）
CY_PX_1920 = 517.261        # cy（主点纵坐标）
CALIB_WH   = (1920, 1080)   # 标定分辨率
CALIB_DATE = "2026-11-04 (来自 S14 stereo_calib.yaml)"
# 畸变 D1 = [-0.02112, 0.09024, -0.00740, -0.00156, 0]，很小：
#   边缘处径向位移约 2~3 px，中心 <1 px，所以默认不做去畸变
#   （做了要每帧多花十几毫秒，收益不抵）。
DIST_COEFFS = (-0.0211200, 0.0902393, -0.0073969, -0.0015599, 0.0)
MARKER_REAL_M = 0.20        # ★ 红圆实际直径：A4=0.20（大场地用这个），A3=0.26

# ── ③ 交接与安全距离（米，自动换像素）───────────────────────
HANDOFF_DIST_M    = 6.5     # 到这个距离停住、交接
HARD_FLOOR_DIST_M = 3.0     # ★ 硬底线：小于它强制刹车+轻退（防撞墙）

# ── ④ 速度与刹车 ───────────────────────────────────────────
V_MAX_MS  = 2.5             # 接近段最大速度（真机先保守，测通再放）
V_MIN_MS  = 0.8
A_DEC_MS2 = 2.0             # 减速度（用于刹车预测）

# ── ⑤ 横向/垂直控制增益（把目标拉到画面中心）───────────────
K_YAW  = 1.2                # 偏航速率增益（rad/s per rad 误差）
K_UP   = 1.5                # 垂直速度增益（m/s per 归一化误差）
MAX_YAW = 0.6               # 偏航速率上限（rad/s）
MAX_UP  = 0.8               # 垂直速度上限（m/s）

# ── ⑥ 跟踪器 ───────────────────────────────────────────────
TRK_MIN_SCORE = 0.28
TRK_MIN_RED   = 0.15
TRK_CTX_SCALE = 1.6
TRK_PAD_BASE  = 40
TRK_PAD_GROW  = 20.0

# ── ⑦ 丢失处理 ─────────────────────────────────────────────
LOSE_SLOW1_S = 1.0          # 丢 >1s 减速到 50%
LOSE_SLOW2_S = 3.0          # 丢 >3s 减速到 20%
LOSE_PAUSE_S = 15.0         # 丢 >15s 悬停等人
LOST_DEBOUNCE = 3           # 连续丢这么多帧才算真丢
GOOD_DEBOUNCE = 3
# ★ 重锁判据用"尺寸一致性"而不是白纸分：
#   红圆直接贴在窗户玻璃上（外面没有白纸）时白纸分只有 0.3 左右，
#   用白纸分当门槛会一直拒绝重锁。而目标不会在零点几秒内变大 2 倍，
#   尺寸才是物理上可靠的连续性约束。
REACQ_SIZE_LO = 0.55        # 候选直径 / 上次中位直径 的下限
REACQ_SIZE_HI = 1.80        #                                    上限
REACQ_D_MIN_PX = 4.5        # ★ 绝对下限：20 cm 红点在 50 m 是 6.3 px，
                            #   4.5 px 已对应 70 m，超出任务范围，
                            #   更小的都是误检噪点（回放里锁到过 3.2px/98m 的斑点）
REACQ_RADIUS_PX = 280.0     # 离最后实测位置多远以内才允许重锁（防止锁到别的红物）
REACQ_STABLE_N  = 3         # 连续几帧稳定才重锁           # ★ 连续成功这么多帧才认为"真的回来了"
                            #   原来成功 1 帧就退出 RECAPTURE，而进入要丢 3 帧，
                            #   形成"丢3→好1→丢3"的 4 帧循环，日志刷屏（现场实测）           # ★ 连续丢这么多帧才判"真丢"（一帧失败只当抖动）
D_SMOOTH_N    = 5           # ★ d_px 中位数滤波窗口（抑制单帧跳变）
D_GATE_MIN    = 18.0        # 就近取候选的门限下限（px）
D_GATE_RATIO  = 0.9         # 门限 = max(D_GATE_MIN, 0.9×上次直径)，防止串到旁边小目标

# ── ⑧ VLM 语义指认 ─────────────────────────────────────────
USE_VLM_HINT = True         # 起飞悬停时调一次，之后不再调
VLM_TIMEOUT  = 30.0        # 指认一次最多等 30 s（缩图后实测 2 s 就出）
VLM_INPUT_W  = 960         # ★ 送模型前把图缩到这个宽度。
                           #   1920x1080 直接送要 90 s（模型切块太多），960 宽只要 2.1 s。
HINT_MAX_DIST = 260
# ── 借自 S14 的安全与稳健性参数 ─────────────────────────
FRAME_MAX_AGE_S = 0.50     # 视频帧超过这么旧就丢弃（S14: GATE_STATUS_MAX_AGE）
ACQ_STABLE_N    = 8        # ACQUIRE 需要连续这么多帧位置稳定才允许确认
ACQ_STABLE_PX   = 40.0     # "位置稳定"的判据：与上一帧中心差小于这个像素
ENV_XY_MAX_M    = 65.0     # 水平硬围栏（相对起飞点）。我们任务要飞 50 m 外
ENV_Z_MIN_M     = 0.30     # 高度下限（相对起飞点）
ENV_Z_MAX_M     = 35.0     # 高度上限
BATT_WARN_V     = 14.0     # 电压低于此告警（6S）
V_BLIND_MS      = 1.0      # 盲进速度（不知道距离，必须慢）
STEP_LEN_M      = 5.0      # ★ 点一下前进一步的默认距离（+/- 键调）
STEP_LEN_MIN_M  = 1.0
STEP_LEN_MAX_M  = 15.0
STEP_SPEED_MS   = 1.0      # 步进速度（m/s）。1.0 兼顾快与安全
STEP_VERTICAL   = False    # 是否同时按点击位置的高度差升降（默认关：更安全）
STEP_MAX_EXTRA_S = 4.0     # 走够距离后的超时余量
BLIND_MAX_S     = 25.0     # 盲进最长时间（超时自动停）
BLIND_FIND_PX   = 120.0    # 跟踪点附近多大范围里出现红点算"找到"
BLIND_FIND_N    = 3        # 连续这么多帧出现才接上
SCAN_YAW_RATE   = 0.22     # 扫视偏航角速度 rad/s（≈12.6°/s）
SCAN_DEG        = 30.0     # 扫视幅度（单边）
CLICK_TOL_PX = 45          # 点击容差：单击点离候选中心多远仍算选中
DISP_MAX_W   = 1280
HINT_TTL_S   = 8.0         # ★ VLM 粗框的有效期。
                           #   粗框是**图像里的固定坐标**，飞机一动就失效；
                           #   过期后按纯 CV 排序，避免被过时提示带偏。        # 显示窗口最大宽度。画面 1920 时缩到 1280，
                           # 否则窗口超出屏幕、鼠标点击位置也会错。        # 最近的候选离 VLM 粗框超过这么多像素 → 认为 VLM 框错了，丢弃

# ═══════════════════════════════════════════════════════════════════
#  派生量（不要手改）
# ═══════════════════════════════════════════════════════════════════
_state = {"FX": FX_PX_1920, "FY": FY_PX_1920,
          "CX": CX_PX_1920, "CY": CY_PX_1920, "W": 1920, "H": 1080}


def set_frame_size(w, h=None):
    """拿到第一帧后调用：按实际分辨率等比换算内参。"""
    h = int(h) if h else int(round(w * CALIB_WH[1] / CALIB_WH[0]))
    kx = w / float(CALIB_WH[0])
    ky = h / float(CALIB_WH[1])
    _state.update({"W": w, "H": h,
                   "FX": FX_PX_1920 * kx, "FY": FY_PX_1920 * ky,
                   "CX": CX_PX_1920 * kx, "CY": CY_PX_1920 * ky})


def dpix_at(dist_m):
    """距离(米) → 红圆像素直径（用 fx）。"""
    return _state["FX"] * MARKER_REAL_M / max(dist_m, 0.1)


class _Tee:
    """把 print 同时写到屏幕和日志文件。"""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for st in self.streams:
            try:
                st.write(data)
            except Exception:
                pass

    def flush(self):
        for st in self.streams:
            try:
                st.flush()
            except Exception:
                pass

    def isatty(self):
        return False


def text(img, s, xy, scale=0.6, color=(235, 235, 235), thick=1):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick,
                cv2.LINE_AA)


# ═══════════════════════════════════════════════════════════════════
#  通信
# ═══════════════════════════════════════════════════════════════════
def _ssh(args, timeout=25):
    return subprocess.run(["ssh", "-i", SSH_KEY,
                           "-o", "StrictHostKeyChecking=no",
                           "-o", "ConnectTimeout=8", DRONE] + args,
                          capture_output=True, text=True, timeout=timeout)


def _scp_from(remote, local, timeout=25):
    return subprocess.run(["scp", "-i", SSH_KEY,
                           "-o", "StrictHostKeyChecking=no",
                           f"{DRONE}:{remote}", str(local)],
                          capture_output=True, text=True, timeout=timeout)


class Link:
    """UDP 双向：发指令、收状态。"""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status = None
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", PORT + 1))
        self.rx.settimeout(0.5)
        self._seq = 0
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _rx_loop(self):
        while True:
            try:
                data, _ = self.rx.recvfrom(2048)
                self.status = json.loads(data.decode())
            except Exception:
                continue

    def send(self, **kw):
        self._seq = max(self._seq + 1, int(time.time() * 1000))
        kw["seq"] = self._seq
        try:
            self.sock.sendto(json.dumps(kw).encode(), (DRONE_IP, PORT))
            return True
        except Exception as e:
            print(f"  ✗ UDP 发送失败 {e}")
            return False

    def send_vel(self, vbx, vby, vz, yr):
        return self.send(type="vel", vbx=float(vbx), vby=float(vby),
                         vz=float(vz), yr=float(yr))


class VideoStream:
    """TCP 长连接收帧（机上 vstream.py 推）。"""

    def __init__(self, host=DRONE_IP, port=VIDEO_PORT):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.latest = None
        self.latest_t = 0.0
        self.count = 0
        self.alive = True
        self.dts = []
        threading.Thread(target=self._loop, daemon=True).start()

    def _recv(self, n):
        buf = b""
        while len(buf) < n:
            b = self.sock.recv(n - len(buf))
            if not b:
                raise ConnectionError("closed")
            buf += b
        return buf

    def _loop(self):
        t_prev = time.time()
        while self.alive:
            try:
                ln = struct.unpack(">I", self._recv(4))[0]
                if ln > 8_000_000:
                    raise ConnectionError("bad len")
                data = self._recv(ln)
            except Exception:
                self.alive = False
                return
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                self.latest = img
                self.latest_t = time.time()   # ★ 必须更新，帧时效判断靠它
                self.count += 1
                now = time.time()
                self.dts.append(now - t_prev)
                self.dts = self.dts[-30:]
                t_prev = now

    @property
    def fps(self):
        if len(self.dts) < 3:
            return 0.0
        m = sum(self.dts) / len(self.dts)
        return 1.0 / m if m > 0 else 0.0


class ReplayStream:
    """把录像文件当成实时视频源（离线复盘用）。

    接口和 VideoStream 一致：latest / latest_t / fps / count / alive。
    内部线程按文件真实帧率喂帧，跑完停在最后一帧（不循环，避免误判"又看到了"）。
    """

    def __init__(self, path, loop=False):
        self.path = str(path)
        self.cap = cv2.VideoCapture(self.path)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开录像 {path}")
        self.file_fps = self.cap.get(cv2.CAP_PROP_FPS) or 10.0
        self.total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.loop = loop
        self.latest = None
        self.latest_t = 0.0
        self.count = 0
        self.alive = True
        self._dt = 1.0 / self.file_fps
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.alive and self.count < self.total:
            ok, fr = self.cap.read()
            if not ok:
                if self.loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                self.alive = False
                return
            self.latest = fr
            self.latest_t = time.time()
            self.count += 1
            time.sleep(self._dt)

    @property
    def fps(self):
        return self.file_fps


def pull_frame_once():
    """兜底：scp 拉一张（1.5 s/帧，只在视频流连不上时用）。"""
    local = Path.home() / "AppData/Local/Temp" / "breach_frame.jpg"
    r = _scp_from("/tmp/cpmv_frame.jpg", local)
    if r.returncode != 0 or not local.exists():
        return None
    return imread_unicode(local)


# ═══════════════════════════════════════════════════════════════════
#  候选检测与 VLM 指认
# ═══════════════════════════════════════════════════════════════════
def detect_reds(bgr, min_area=4):
    """红点检测：用按真实画面调好的版本（HSV 收紧 + 形状 + 白纸环带）。

    旧的松阈值在室内画面里出 626 个候选（窗帘/木箱全中）；
    新版实测 6 个候选，且白纸红点排第一。
    """
    from mcpv.detect_live import detect_reds as _d
    return _d(bgr)


def vlm_hint(frame):
    """调用一次 VLM：返回粗框中心与描述。失败返回 None。"""
    try:
        from mcpv.vlm import ask_crop
    except Exception as e:
        print(f"  ⚠ 导入 VLM 失败：{type(e).__name__}")
        return None
    prompt = ("Please provide the bounding box coordinate of the region "
              "this sentence describes: <ref>红色圆点</ref>")
    # ★ 缩图：坐标是 0~1000 归一的，缩了不影响结果，但快 40 倍
    h0, w0 = frame.shape[:2]
    if w0 > VLM_INPUT_W:
        sc = VLM_INPUT_W / float(w0)
        frame = cv2.resize(frame, (VLM_INPUT_W, int(h0 * sc)),
                           interpolation=cv2.INTER_AREA)
    try:
        raw, _dt = ask_crop(frame, prompt, timeout=VLM_TIMEOUT)
    except Exception as e:
        print(f"  ⚠ VLM 调不通：{type(e).__name__}（Ollama 起了吗？）")
        return None

    import re
    box = None
    m = re.search(r"<box>\s*([\d\s,]+?)\s*</box>", raw or "")
    if m:
        n = [int(x) for x in re.findall(r"\d+", m.group(1))]
        if len(n) >= 4:
            box = n[:4]
    if box is None:
        runs = [int(x) for x in re.findall(r"\d+", raw or "")]
        if len(runs) >= 4 and all(0 <= v <= 1000 for v in runs[-4:]):
            box = runs[-4:]
    if not box:
        print(f"  ⚠ VLM 没给出可用框，原文：{(raw or '')[:60]}")
        return None
    # 归一到 0~1000，再按**原始**尺寸换算（frame 可能已被缩小）
    return {"cx": (box[0] + box[2]) / 2 / 1000.0 * w0,
            "cy": (box[1] + box[3]) / 2 / 1000.0 * h0,
            "box": box, "raw": raw}


def _vlm_worker(frame, slot):
    """后台线程里跑 VLM，结果写进 slot，绝不阻塞主循环。"""
    try:
        slot["hint"] = vlm_hint(frame)
    except Exception as e:
        print(f"  ⚠ VLM 线程异常 {type(e).__name__}")
        slot["hint"] = None
    slot["done"] = True


def vlm_pick_candidate(frame, cands, n_max=4, tile=220):
    """把前 n_max 个候选裁成并排编号图，让 VLM 选"哪一个是红色圆形标记"。

    返回 (选中的候选 dict 或 None, 原文, 耗时秒)。
    ★ 这是实测唯一可靠的 VLM 用法：在有限选项里做比较判断。
    """
    if len(cands) < 2:
        return None, "", 0.0      # 只有一个候选时没必要问
    from mcpv.vlm import ask_crop
    picks = cands[:n_max]
    tiles = []
    for i, q in enumerate(picks, 1):
        h = max(48, int(q["d"] * 1.5))
        cx, cy = int(q["cx"]), int(q["cy"])
        x0, y0 = max(0, cx - h), max(0, cy - h)
        x1 = min(frame.shape[1], cx + h)
        y1 = min(frame.shape[0], cy + h)
        t = frame[y0:y1, x0:x1]
        if t.size == 0:
            continue
        t = cv2.resize(t, (tile, tile), interpolation=cv2.INTER_LINEAR)
        cv2.rectangle(t, (0, 0), (tile - 1, tile - 1), (255, 255, 255), 2)
        cv2.putText(t, str(i), (8, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                    (0, 255, 255), 3)
        tiles.append(t)
    if len(tiles) < 2:
        return None, "", 0.0
    grid = np.hstack(tiles)
    qtext = (f"这张图里并排着 {len(tiles)} 块局部画面，每块左上角有编号。"
             "只有其中一块是从远处拍到的【红色圆形标记】（人工贴的靶标），"
             "其余是墙面、砖、灯、地面等。哪一个编号是红色圆形标记？只回答一个数字。")
    t0 = time.time()
    try:
        ans, _ = ask_crop(grid, qtext, timeout=VLM_TIMEOUT)
    except Exception as exc:
        print(f"  ⚠ VLM 对比失败：{type(exc).__name__}")
        return None, "", time.time() - t0
    dt = time.time() - t0
    import re
    m = re.search(r"\d+", ans or "")
    if not m:
        return None, ans or "", dt
    k = int(m.group(0))
    if not (1 <= k <= len(picks)):
        return None, ans or "", dt
    return picks[k - 1], ans or "", dt


def _vlm_pick_worker(frame, cands, slot):
    best, raw, dt = vlm_pick_candidate(frame, cands)
    slot["pick"] = best
    slot["raw"] = raw
    slot["dt"] = dt
    slot["done"] = True


def rank_by_hint(cands, hint):
    if not cands:
        return []
    if hint is None:
        return [(0.0, c) for c in cands]
    dmax = max(1e-6, max(math.hypot(c["cx"] - hint["cx"], c["cy"] - hint["cy"])
                        for c in cands))
    out = []
    for c in cands:
        d = math.hypot(c["cx"] - hint["cx"], c["cy"] - hint["cy"])
        out.append((0.85 * (1 - d / dmax) + 0.15 * min(1.0, c["d"] / 30.0), c))
    out.sort(key=lambda x: -x[0])
    return out


# ═══════════════════════════════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════════════════════════════
STEP_LEN_STATE = [STEP_LEN_M]      # 当前步长（+/- 键改）


def step_len_adj(_dummy, delta):
    """调整步长并打印。"""
    v = STEP_LEN_STATE[0] + delta
    v = max(STEP_LEN_MIN_M, min(STEP_LEN_MAX_M, v))
    STEP_LEN_STATE[0] = v
    print(f"  ★ 步长 = {v:.1f} m")


RUN_TAG = [""]          # 本次运行的标签，用于统一命名
_ORIG_STDOUT = sys.stdout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--marker-cm", type=float, default=0)
    ap.add_argument("--no-vlm", action="store_true")
    ap.add_argument("--record", default="",
                    help="录像输出路径；不给就自动生成带时间戳的文件")
    ap.add_argument("--no-record", action="store_true", help="不录像")
    ap.add_argument("--auto", action="store_true",
                    help="自动锁定稳定的头号候选（离线回放/回归测试用）")
    ap.add_argument("--replay", default="",
                    help="用录像文件当视频源（离线复盘，不连飞机）")
    ap.add_argument("--headless", action="store_true",
                    help="不开窗口，跑 --seconds 秒后退出（用于验证主循环）")
    ap.add_argument("--seconds", type=float, default=0,
                    help="配合 --headless，跑多少秒")
    a = ap.parse_args()

    if a.marker_cm > 0:
        global MARKER_REAL_M
        MARKER_REAL_M = a.marker_cm / 100.0
    use_vlm = USE_VLM_HINT and not a.no_vlm

    print("=" * 64)
    print("  破窗前置：红点跟踪 → 接近 → 交接")
    print("    1 = 真飞（螺旋桨已装、场地清空）")
    print("    2 = dryrun（只算指令、不发给飞机）")
    print("=" * 64)
    if a.live:
        live = True
    elif a.dry:
        live = False
    else:
        live = input("选 1 或 2: ").strip() == "1"
    print(f"模式: {'★ 真飞' if live else 'dryrun'}")

    if not a.replay and "OK" not in _ssh(["echo OK"]).stdout:
        print("✗ 连不上飞机")
        return 1

    # 起视频流
    def _try_stream(wait=8.0):
        try:
            v = VideoStream()
            t_stream = time.time()
            while v.latest is None and time.time() - t_stream < wait:
                time.sleep(0.1)
            return v if v.latest is not None else None
        except Exception as exc:
            print(f"  ✗ 视频流连不上：{exc}")
            return None

    if a.replay:
        # ★ 离线回放：不连飞机、不启推流
        print(f"  回放模式：{a.replay}")
        vs = ReplayStream(a.replay)
        t0r = time.time()
        while vs.latest is None and time.time() - t0r < 10:
            time.sleep(0.05)
        if vs.latest is None:
            print("  ✗ 录像读不出帧")
            return 1
        print(f"  录像 {vs.total} 帧 @ {vs.file_fps:.1f} fps")
    else:
        vs = _try_stream()
    if vs is None:
        # ★ 飞机重启后 /tmp 被清空，grabd/vstream 都不在了 —— 自动拉起
        print("  机上推流没起，自动启动（hd 模式）…")
        _ssh(["bash ~/cpmvbyhuang/start_grabd.sh hd"], timeout=40)
        for k in range(3):
            time.sleep(4)
            vs = _try_stream(6.0)
            if vs is not None:
                print("  推流已恢复")
                break
        if vs is None:
            print("  ⚠ 自动启动失败，退回 scp 单帧（很慢，1.5 s/帧）")

    frame = vs.latest if (vs and vs.latest is not None) else pull_frame_once()
    if frame is None:
        print("✗ 拿不到画面")
        return 1
    H, W = frame.shape[:2]
    set_frame_size(W, H)
    D_HANDOFF_PX = dpix_at(HANDOFF_DIST_M)
    D_FLOOR_PX = dpix_at(HARD_FLOOR_DIST_M)
    print(f"画面 {W}x{H}   红圆 {MARKER_REAL_M*100:.0f} cm")
    print(f"  内参 fx={_state['FX']:.1f} fy={_state['FY']:.1f} "
          f"cx={_state['CX']:.1f} cy={_state['CY']:.1f}   ({CALIB_DATE})")
    print(f"  交接 {HANDOFF_DIST_M:.1f} m → d_px {D_HANDOFF_PX:.1f}   "
          f"硬底线 {HARD_FLOOR_DIST_M:.1f} m → d_px {D_FLOOR_PX:.1f}")
    if vs:
        time.sleep(0.6)
        print(f"  视频流 {vs.fps:.1f} fps")

    link = Link()
    trk = TemplateTracker(ctx_scale=TRK_CTX_SCALE, base_pad=TRK_PAD_BASE,
                          pad_grow=TRK_PAD_GROW)
    trk.MIN_SCORE = TRK_MIN_SCORE
    trk.MIN_RED_RATIO = TRK_MIN_RED

    state = "ACQUIRE"
    cands, ranked, hint = [], [], None
    sel = {"cand": None}
    vlm_state = {"done": False, "hint": None, "pick": None,
                 "raw": "", "dt": 0.0}
    vlm_mark = [None]      # VLM 选中的候选位置（画星标用）
    confirm_pos = [None]   # 人工确认时的目标位置（VLM 事后核对用）
    vlm_log = {}           # 写进交接 JSON 的 VLM 记录
    vlm_checked = [False]
    vlm_started = [False]
    vlm_consumed = [False]
    hint_t = [0.0]
    miss = [0]            # 连续丢失帧数（去抖用）
    good_n = [0]          # 连续成功帧数（退出重捕获的去抖）
    d_hist = []           # 最近几帧的 d_px（中位数滤波）
    stab = [0, None]      # [连续稳定帧数, 上一帧最优候选中心]
    origin = [None]       # 起飞点（第一次拿到的 pos），用于世界系围栏
    scan = {"on": False, "t0": 0.0}   # G 键扫视
    step = {"on": False, "dir": (0.0, 0.0), "vz": 0.0,
            "t0": 0.0, "len": 0.0, "origin": None}
    blind = {"pix": None, "t0": 0.0, "hit_n": 0}   # 盲进
    reacq = {"n": 0, "c": None}    # 检测兜底重锁的稳定计数
    manual_vel = [(0.0, 0.0, 0.0, 0.0)]
    manual_until = [0.0]
    manual = False
    last_sent = 0.0
    last_print = [0.0]
    lost_s = 0.0
    warn = ""
    trans = []
    t0 = time.time()
    last_frame = None
    fps_ema = 0.0
    t_frame = time.time()
    last_frame_t = time.time()
    land_t = [0.0]
    t_track = [0.0]

    # ★ 默认录像：给上级看成果时不能忘记录
    rec_path = None
    if not a.no_record:
        rec_path = a.record or str(
            ROOT / "eval" / "demo_out" / f"run_{datetime.now():%Y%m%d_%H%M%S}.mp4")
        Path(rec_path).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(rec_path, cv2.VideoWriter_fourcc(*"mp4v"),
                                 10, (W, H))
        if writer.isOpened():
            # ★ 同时把控制台输出存成日志（VLM 原文、状态转移、每条指令都在里面）
            RUN_TAG[0] = Path(rec_path).stem.replace("run_", "")
            log_path = Path(rec_path).with_suffix(".log")
            try:
                _log_fh = open(log_path, "w", encoding="utf-8", buffering=1)
                sys.stdout = _Tee(_ORIG_STDOUT, _log_fh)
                print(f"  ● 录像中 → {rec_path}")
                print(f"  ● 日志   → {log_path}")
            except Exception as e:
                print(f"  ⚠ 日志文件打开失败 {type(e).__name__}，只输出到屏幕")
                print(f"  ● 录像中 → {rec_path}")
        else:
            print(f"  ✗ 录像打不开 {rec_path}，继续但不录")
            writer = None
            rec_path = None
    else:
        writer = None

    def enter(ns, why=""):
        nonlocal state
        if ns != state:
            trans.append({"t": round(time.time() - t0, 1), "from": state,
                          "to": ns, "why": why})
            print(f"  [{time.time()-t0:5.1f}s] {state:9s} -> {ns:9s}  {why}")
            state = ns

    print("\n操作：单击选候选 / 回车确认 / WASD RF QE 接管 / T 恢复 / "
          "空格急停 / X 退出 / L L 降落")
    if not a.headless:
        cv2.namedWindow("breach", cv2.WINDOW_AUTOSIZE)

    def do_confirm():
        """确认当前选中的候选，初始化跟踪器并进入 TRACK。"""
        nonlocal manual          # ★ 必须声明：否则下面的 manual=False 只改局部变量
        c0 = sel["cand"]

        # ★ 盲进：没检出候选、但点了一个像素点 → 朝那里慢慢飞过去
        if c0 is None:
            bp_s = blind["pix"]
            if bp_s is None:
                print("  还没选目标：单击红点（或点一个大概位置走盲进）")
                return
            # ★ 点一下走固定距离（人眼兜底）
            ex_s = (bp_s[0] - _state["CX"]) / _state["FX"]
            ey_s = (bp_s[1] - _state["CY"]) / _state["FY"]
            ang = math.atan(ex_s)              # 水平偏角，右为正
            vbx = STEP_SPEED_MS * math.cos(ang)
            vby = -STEP_SPEED_MS * math.sin(ang)   # 机体系 +y 向左
            vz_s = 0.0
            if STEP_VERTICAL:
                vz_s = max(-0.5, min(0.5,
                                     -STEP_SPEED_MS * math.sin(math.atan(ey_s))))
            step.update({"on": True, "dir": (vbx, vby), "vz": vz_s,
                         "t0": time.time(), "len": STEP_LEN_STATE[0], "origin": None})
            st0 = link.status
            if st0 and st0.get("pos"):
                step["origin"] = tuple(st0["pos"])
            manual = False
            print(f"  ★ 步进 {STEP_LEN_STATE[0]:.1f} m 朝 ({bp_s[0]:.0f},{bp_s[1]:.0f})"
                  f"（偏角 {math.degrees(ang):+.1f}°）  速度 {STEP_SPEED_MS:.1f} m/s")
            enter("STEP", f"步进 {STEP_LEN_STATE[0]:.1f} m")
            return

        if stab[0] < ACQ_STABLE_N and state == "ACQUIRE":
            print(f"  目标还不稳定（{stab[0]}/{ACQ_STABLE_N} 帧），再稳一下再确认")
            return
        # 用当前帧重新对齐一次（选中到确认之间目标会动）
        if cands:
            q_near = min(cands, key=lambda q: math.hypot(
                q["cx"] - c0["cx"], q["cy"] - c0["cy"]))
            if math.hypot(q_near["cx"] - c0["cx"],
                          q_near["cy"] - c0["cy"]) < 40:
                c0 = q_near
        blind["pix"] = None        # ★ 已锁定目标，清掉步进点
        confirm_pos[0] = (c0["cx"], c0["cy"])
        trk.init(frame, (c0["cx"], c0["cy"]),
                 half_size=max(8, int(c0["d"] * 1.1)), d_px=c0["d"])
        t_track[0] = time.time()
        manual = False                   # ★ 新目标：退出手动接管
        miss[0] = 0                      # ★ 清丢失计数
        good_n[0] = 0
        d_hist.clear()                   # ★ 清上一次的尺度历史
        d_hist.append(float(c0["d"]))
        enter("TRACK", f"人工确认 d={c0['d']:.1f}px")

    def on_mouse(e, x, y, flags, param):
        if state not in ("ACQUIRE", "PAUSE"):
            return
        if e != cv2.EVENT_LBUTTONDOWN:
            return
        dsc = disp_scale[0]
        x, y = x / dsc, y / dsc        # 窗口坐标 → 原图坐标

        pool = cands if cands else show_cands   # ★ 用实时候选，不用旧列表
        hit = None
        for cd in pool:                # 先按"点在框内"找
            bx, by, bw, bh = cd["box"]
            if bx - 10 <= x <= bx + bw + 10 and by - 10 <= y <= by + bh + 10:
                hit = cd
                break
        if hit is None and pool:       # 容错：取离点击最近的候选
            order = sorted(pool, key=lambda q: math.hypot(q["cx"] - x,
                                                          q["cy"] - y))
            dist0 = math.hypot(order[0]["cx"] - x, order[0]["cy"] - y)
            if dist0 <= CLICK_TOL_PX:
                hit = order[0]
                if len(order) > 1:
                    d1 = math.hypot(order[1]["cx"] - x, order[1]["cy"] - y)
                    if d1 - dist0 < 30:   # 两个都挺近 → 容易选错，提醒一下
                        print(f"  ⚠ 两个候选都在附近（{dist0:.0f}px / {d1:.0f}px），"
                              f"取了更近的 {order[0]['d']:.1f}px 那个；"
                              f"要选另一个请点得更准")
            else:
                sel["cand"] = None      # ★ 点空白=取消选中，否则回车会确认旧候选
                blind["pix"] = (x, y)
                print(f"  没命中候选（最近的在 {dist0:.0f}px 外）→ 记为步进点 "
                      f"({x:.0f},{y:.0f})，按回车开始慢慢前进")
                return
        if hit is None:
            sel["cand"] = None
            blind["pix"] = (x, y)
            print(f"  当前没有候选 → 步进点 ({x:.0f},{y:.0f})，按回车前进")
            return
        blind["pix"] = None        # 命中候选时清掉盲进点

        sel["cand"] = hit
        print(f"  选中 d={hit['d']:.1f}px ({hit['cx']:.0f},{hit['cy']:.0f})"
              f"  → 按回车确认")

    show_cands = []
    disp_scale = [1.0]
    if not a.headless:
        cv2.setMouseCallback("breach", on_mouse)

    if live:
        print("\n⚠ 真飞：确认螺旋桨已装、周围清空")
        if input("输入 YES 继续: ").strip() != "YES":
            return 0
        link.send(type="takeoff", z=1.5)
        print("  起飞已下发，等 10 秒…")
        time.sleep(10)

    stale_n = 0
    while True:
        now0 = time.time()
        # ★ 退出判据放在最前面：任何 continue 都不能绕过它
        #   （原来 --seconds 的判断在循环末尾，一旦某条 continue 常真，就会静默死循环）
        if a.headless and a.seconds > 0 and (now0 - t0) > a.seconds:
            print(f"\n[headless] 跑满 {a.seconds}s，退出")
            break

        # ── 取帧 ─────────────────────────────────────
        if vs and vs.latest is not None:
            frame = vs.latest
        else:
            if time.time() - t_frame > 1.6:
                f2 = pull_frame_once()
                t_frame = time.time()
                if f2 is not None:
                    frame = f2

        # ★ 没有新帧时也必须 pump 一次 GUI，否则 Windows 判"未响应"。
        # ★ 帧时效（借自 S14 GATE_STATUS_MAX_AGE）：视频流卡住时不许拿旧帧做判断
        age = (now0 - vs.latest_t) if (vs and vs.latest is not None) else 0.0
        if age > FRAME_MAX_AGE_S:
            stale_n += 1
            if stale_n % 200 == 1:      # 别刷屏，约每 2 秒一条
                print(f"  ⚠ 视频帧过期 {age:.2f}s（>{FRAME_MAX_AGE_S}s），跳过"
                      f"  已连续 {stale_n} 帧")
            if not a.headless:
                cv2.waitKey(1)
            time.sleep(0.01)
            continue
        stale_n = 0

        if frame is None or (last_frame is not None and frame is last_frame):
            if not a.headless:
                cv2.waitKey(1)
            time.sleep(0.01)
            continue

        now = time.time()
        warn = ""          # ★ 每帧复位，否则 "TOO CLOSE" 会一直挂在屏上
        dt = max(1e-3, min(1.0, now - last_frame_t))
        last_frame = frame
        last_frame_t = now
        fps_ema = 0.9 * fps_ema + 0.1 * (1.0 / dt) if fps_ema else 1.0 / dt

        cands = detect_reds(frame)

        vis = frame.copy()

        # ══ 排序与候选绘制：**任何状态都做** ══
        #   原来这段只在 ACQUIRE 分支里，导致进入 PAUSE（丢失 15 s）后
        #   既不画框、ranked 也不更新，鼠标只能命中几秒前的旧坐标，
        #   表现为"点了没反应 / 60px 内没有候选"（实测踩过）。
        if state != "HANDOFF":
            ranked = rank_by_hint(cands, hint)
            show_cands = [c for _, c in ranked][:6]
            for k, c in enumerate(show_cands):
                col = (80, 240, 90) if c is sel["cand"] else (
                    (60, 220, 255) if k == 0 else (150, 170, 190))
                x, y, w, h = c["box"]
                pad = max(6, int(c["d"] * 0.6))
                cv2.rectangle(vis, (x - pad, y - pad),
                              (x + w + pad, y + h + pad), col, 2)
                dist_m = (_state["FX"] * MARKER_REAL_M / c["d"]
                          if c["d"] > 0 else 0.0)
                text(vis, f"{k+1}: {c['d']:.0f}px {dist_m:.1f}m",
                     (x - pad, y - pad - 8), 0.6, col, 2)
            if show_cands and state == "ACQUIRE":
                b = show_cands[0]
                dm = (_state["FX"] * MARKER_REAL_M / b["d"]) if b["d"] > 0 else 0
                text(vis, f"BEST  {b['d']:.1f} px   ~{dm:.1f} m   "
                          f"(handoff {D_HANDOFF_PX:.0f} px = "
                          f"{HANDOFF_DIST_M:.1f} m)",
                     (12, H - 120), 0.75, (120, 230, 120), 2)

        # ── 状态机 ────────────────────────────────────
        if state == "ACQUIRE":
            # ★ 稳定帧计数（借自 S14 GATE_ACQ_MIN_N）：
            #   不许一帧就定目标。最优候选与上一帧差 < ACQ_STABLE_PX 才算稳定。
            if cands:
                top = cands[0]
                prev = stab[1]
                if prev is not None and math.hypot(
                        top["cx"] - prev[0], top["cy"] - prev[1]) < ACQ_STABLE_PX:
                    stab[0] = min(ACQ_STABLE_N, stab[0] + 1)
                else:
                    stab[0] = 1
                stab[1] = (top["cx"], top["cy"])
            else:
                stab[0] = 0
                stab[1] = None
            acq_ready = stab[0] >= ACQ_STABLE_N

            # ★ VLM 必须放后台线程：一次生成要几秒到几十秒，
            #   同步调用会让 imshow/waitKey 停住，窗口被判"未响应"（实测踩过）。
            #   而且等目标稳定后再送图：模糊/晃动帧会让 VLM 框错（实测 4 次错 3 次）。
            # ★ 候选 ≥2 才问（只有一个时没必要，也省 1 秒）
            if use_vlm and not vlm_started[0] and acq_ready and len(cands) >= 2:
                vlm_started[0] = True
                th = threading.Thread(target=_vlm_pick_worker,
                                      args=(frame.copy(),
                                            [dict(c) for c in cands[:4]],
                                            vlm_state),
                                      daemon=True)
                th.start()
                print(f"  [VLM] 后台对比 {min(len(cands), 4)} 个候选…"
                      f"（并排编号，只回答数字）")

            if vlm_state["done"] and not vlm_consumed[0]:
                vlm_consumed[0] = True
                pick = vlm_state.get("pick")
                raw = vlm_state.get("raw") or ""
                dtv = vlm_state.get("dt") or 0.0
                if pick is not None:
                    hint = {"cx": pick["cx"], "cy": pick["cy"],
                            "raw": raw, "d": pick["d"]}
                    vlm_mark[0] = (pick["cx"], pick["cy"])
                    print(f"  [VLM] 选中候选 d={pick['d']:.1f}px "
                          f"({pick['cx']:.0f},{pick['cy']:.0f})  "
                          f"白纸分{pick.get('paper', 0):.2f}  {dtv:.1f}s")
                    print(f"        原文：{raw[:50]}")
                    print(f"        ≈{_state['FX']*MARKER_REAL_M/max(pick['d'],0.1):.1f} m")
                else:
                    hint = None
                    print(f"  [VLM] 没选出（原文「{raw[:30]}」）→ 按纯 CV 排序")
                # 现在 VLM 给的是"选中了哪个候选"，不是粗框坐标，
                # 所以不再需要"离红点太远就丢弃"的校验。
                if hint is not None:
                    hint_t[0] = now
            # ★ 粗框过期：飞机在动，图像坐标会失效，过期后按纯 CV 排序
            if hint is not None and (now - hint_t[0]) > HINT_TTL_S:
                print(f"  [VLM] 粗框已过期（>{HINT_TTL_S:.0f}s），改用纯 CV 排序")
                hint = None
            if show_cands:
                b = show_cands[0]
                dm = (_state["FX"] * MARKER_REAL_M / b["d"]) if b["d"] > 0 else 0
                text(vis, f"BEST  {b['d']:.1f} px   ~{dm:.1f} m   "
                          f"(handoff {D_HANDOFF_PX:.0f} px = {HANDOFF_DIST_M:.1f} m)",
                     (12, H - 120), 0.75, (120, 230, 120), 2)
            if sel["cand"] is not None:
                text(vis, "已选中 → 按回车确认", (12, H - 92), 0.7,
                     (60, 220, 255), 2)

            # ★ --auto：稳定后自动锁定头号候选（无人值守的离线回放用）
            if a.auto and acq_ready and show_cands:
                sel["cand"] = show_cands[0]
                print(f"  [auto] 自动锁定 d={show_cands[0]['d']:.1f}px "
                      f"白纸分{show_cands[0]['paper']:.2f}")
                do_confirm()

        elif state == "STEP":
            # 走够 L 米就停：实飞用 pos 位移判断，干跑按时长
            el = now - step["t0"]
            done_len = False
            if step["origin"] is not None and link.status:
                pn = link.status.get("pos")
                if pn:
                    moved = math.sqrt(sum((pn[i] - step["origin"][i]) ** 2
                                          for i in range(3)))
                    done_len = moved >= step["len"]
                    if not done_len:
                        text(vis, f"STEP  {moved:.1f}/{step['len']:.1f} m",
                             (12, H - 120), 0.8, (0, 200, 255), 2)
            if not done_len:
                done_len = el >= step["len"] / max(STEP_SPEED_MS, 0.1)
            if done_len or el > step["len"] / max(STEP_SPEED_MS, 0.1) \
                    + STEP_MAX_EXTRA_S:
                if live:
                    link.send_vel(0, 0, 0, 0)
                step["on"] = False
                enter("ACQUIRE",
                      f"步进完成（{el:.1f}s），请重新点选/确认")
            elif not manual and now - last_sent > 0.1:
                last_sent = now
                if live:
                    link.send_vel(step["dir"][0], step["dir"][1], step["vz"], 0)
                if now - last_print[0] > 0.5:
                    last_print[0] = now
                    print(f"    {'DRY ' if not live else ''}步进中 "
                          f"{el:4.1f}s / {step['len']/STEP_SPEED_MS:.1f}s  "
                          f"vel=({step['dir'][0]:+.1f},{step['dir'][1]:+.1f})")
            if step["on"]:
                text(vis, f"STEP  {el:.1f}s / "
                          f"{step['len']/max(STEP_SPEED_MS,0.1):.1f}s",
                     (12, H - 120), 0.8, (0, 200, 255), 2)

        elif state == "BLIND":
            # ★ 盲进：不知道距离，只按固定慢速往前飞，同时盯住点选的那块纹理。
            ok, cen, score = trk.update(frame, dt, d_est=None)
            if not ok:
                cen = trk.predict(frame, dt)
            # 在跟踪点附近找红点：连续 BLIND_FIND_N 帧出现就接上
            found = None
            if cands:
                near = min(cands, key=lambda c: math.hypot(
                    c["cx"] - cen[0], c["cy"] - cen[1]))
                if math.hypot(near["cx"] - cen[0],
                              near["cy"] - cen[1]) <= BLIND_FIND_PX:
                    found = near
            blind["hit_n"] = blind["hit_n"] + 1 if found else 0

            if found is not None and blind["hit_n"] >= BLIND_FIND_N:
                d0 = float(found["d"])
                trk.init(frame, (found["cx"], found["cy"]),
                         half_size=max(8, int(d0 * 1.1)), d_px=d0)
                t_track[0] = time.time()
                miss[0] = 0
                good_n[0] = 0
                d_hist.clear()
                d_hist.append(d0)
                sel["cand"] = found
                if live:
                    link.send_vel(0, 0, 0, 0)
                enter("APPROACH",
                      f"盲进找到红点 d={d0:.1f}px "
                      f"≈{_state['FX']*MARKER_REAL_M/d0:.1f}m，自动接上跟踪")
            else:
                el_b = now - blind["t0"]
                if el_b > BLIND_MAX_S:
                    if live:
                        link.send_vel(0, 0, 0, 0)
                    enter("ACQUIRE", f"盲进 {el_b:.0f}s 超时，停下重新选")
                if cen is not None:
                    ex_b = (cen[0] - _state["CX"]) / _state["FX"]
                    ey_b = (cen[1] - _state["CY"]) / _state["FY"]
                    yr_b = max(-MAX_YAW, min(MAX_YAW, -K_YAW * ex_b))
                    vz_b = max(-MAX_UP, min(MAX_UP, -K_UP * ey_b))
                    if manual:
                        pass                     # 手动时由循环末尾统一发
                    elif now - last_sent > 0.1:
                        last_sent = now
                        if live:
                            link.send_vel(V_BLIND_MS, 0.0, vz_b, yr_b)
                        if now - last_print[0] > 0.5:
                            last_print[0] = now
                            what = (f"发现候选 d={found['d']:.1f}px "
                                    f"({blind['hit_n']}/{BLIND_FIND_N})"
                                    if found else "还没看到红点")
                            print(f"    {'DRY ' if not live else ''}盲进 "
                                  f"fwd={V_BLIND_MS:.1f} {el_b:4.1f}s  {what}")
                    cv2.drawMarker(vis, (int(cen[0]), int(cen[1])),
                                   (60, 200, 255), cv2.MARKER_TILTED_CROSS,
                                   28, 2)
                    text(vis, f"BLIND  {el_b:.1f}s  {V_BLIND_MS:.1f} m/s",
                         (12, H - 120), 0.8, (60, 200, 255), 2)

        elif state in ("TRACK", "APPROACH", "RECAPTURE"):
            ok, cen, score = trk.update(
                frame, dt, d_est=(d_hist[-1] if d_hist else None))
            measured = ok          # ★ True=实测  False=预测
            if ok:
                miss[0] = 0
                good_n[0] += 1
                lost_s = 0.0
                # ★ 取 d_px：门限随目标大小自适应。
                #   固定 40 px 会把旁边 12 px 的红绿灯也算进来，
                #   导致距离在 6~25 m 之间乱跳（现场日志实测）。
                d_new = 0.0
                if cands:
                    near = min(cands, key=lambda c: math.hypot(
                        c["cx"] - cen[0], c["cy"] - cen[1]))
                    dn = math.hypot(near["cx"] - cen[0], near["cy"] - cen[1])
                    gate = max(D_GATE_MIN,
                               D_GATE_RATIO * (d_hist[-1] if d_hist else 24.0))
                    if dn <= gate:
                        d_new = near["d"]
                if d_new > 0:
                    d_hist.append(d_new)
                    d_hist[:] = d_hist[-D_SMOOTH_N:]
                d_now = float(np.median(d_hist)) if d_hist else 0.0
                if state == "RECAPTURE" and good_n[0] >= GOOD_DEBOUNCE:
                    enter("APPROACH", f"重捕获 score={score:.2f} 连续{good_n[0]}帧")
            else:
                miss[0] += 1
                good_n[0] = 0
                cen = trk.predict(frame, dt)
                lost_s = trk.lost_s
                # ★ 丢了也沿用已平滑的尺度，不要清 0，否则距离会跳
                d_now = float(np.median(d_hist)) if d_hist else 0.0
                # ★ 去抖：连续丢 LOST_DEBOUNCE 帧才算真丢。
                #   现场日志里"丢一帧就切 RECAPTURE、下一帧又切回来"
                #   来回抖了十几次，全是噪声。
                if miss[0] >= LOST_DEBOUNCE and state in ("APPROACH", "TRACK"):
                    enter("RECAPTURE", f"连续丢 {miss[0]} 帧")
                # ★ 检测兜底重锁：模板配不上时，用检测结果把目标找回来。
                #   现场录像里 RECAPTURE 占了 19.7%、PAUSE 又占 19.7%，
                #   而检测器其实每帧都检得到红点 —— 白白在找目标。
                if (state == "RECAPTURE" and cands
                        and getattr(trk, "last_good", None) is not None):
                    lg = trk.last_good
                    # 位置：离最后实测点不能太远
                    # 尺寸：和最近几帧的中位直径接近（防止锁到别的红物体）
                    dref = float(np.median(d_hist)) if d_hist else None
                    pool2 = []
                    for c in cands:
                        if math.hypot(c["cx"] - lg[0],
                                      c["cy"] - lg[1]) > REACQ_RADIUS_PX:
                            continue
                        if c["d"] < REACQ_D_MIN_PX:
                            continue          # 太小，不可能是 20 cm 目标
                        if dref is not None and not (
                                REACQ_SIZE_LO * dref <= c["d"]
                                <= REACQ_SIZE_HI * dref):
                            continue
                        pool2.append(c)
                    if pool2:
                        # 多个都合格时，优先白纸分高的（有白纸当然更好），
                        # 但**不要求**有白纸
                        best2 = max(pool2, key=lambda c: (
                            c.get("paper", 0.0), c["fill"]))
                        if (reacq["c"] is not None
                                and math.hypot(best2["cx"] - reacq["c"][0],
                                               best2["cy"] - reacq["c"][1]) < 25):
                            reacq["n"] += 1
                        else:
                            reacq["n"] = 1
                        reacq["c"] = (best2["cx"], best2["cy"])
                        if reacq["n"] >= REACQ_STABLE_N:
                            d0 = float(best2["d"])
                            trk.init(frame, (best2["cx"], best2["cy"]),
                                     half_size=max(10, int(d0 * 1.1)), d_px=d0)
                            t_track[0] = time.time()
                            miss[0] = 0
                            good_n[0] = 0
                            d_hist.clear()
                            d_hist.append(d0)
                            reacq["n"] = 0
                            reacq["c"] = None
                            if live:
                                link.send_vel(0, 0, 0, 0)
                            enter("APPROACH",
                                  f"检测兜底重锁 d={d0:.1f}px "
                                  f"白纸分{best2['paper']:.2f} "
                                  f"≈{_state['FX']*MARKER_REAL_M/d0:.1f}m")
                    else:
                        reacq["n"] = 0
                        reacq["c"] = None

                if lost_s > LOSE_PAUSE_S:
                    enter("PAUSE", f"丢失 {lost_s:.1f}s，悬停等人")
                    if live:
                        link.send_vel(0, 0, 0, 0)

            if state == "TRACK" and score >= TRK_MIN_SCORE:
                if t_track[0] == 0.0:
                    t_track[0] = now
                elif now - t_track[0] > 0.8:
                    enter("APPROACH", "跟踪稳定")

            if state in ("APPROACH", "RECAPTURE") and d_now > 0:
                dist_m = _state["FX"] * MARKER_REAL_M / d_now
                # 刹车允许速度
                v_allow = math.sqrt(max(0.0, 2 * A_DEC_MS2 *
                                       (dist_m - HANDOFF_DIST_M)))
                if lost_s > LOSE_SLOW2_S:
                    v_allow *= 0.2
                elif lost_s > LOSE_SLOW1_S:
                    v_allow *= 0.5
                if manual:
                    v_allow = 0.0   # 手动期间不发自动前进指令
                v_fwd = max(V_MIN_MS if dist_m > HANDOFF_DIST_M * 1.05 else 0.0,
                            min(V_MAX_MS, v_allow))

                # ★ 把目标拉到**光轴**（真实主点），不是画面几何中心。
                #   主点 cx=932.3 而 W/2=960，差 27.7 px —— 用 W/2 会让
                #   目标明明在光轴上却被判成偏了 1°，长期会把飞机带偏。
                ex = (cen[0] - _state["CX"]) / _state["FX"]   # 水平角正切
                ey = (cen[1] - _state["CY"]) / _state["FY"]   # 垂直角正切
                yr = max(-MAX_YAW, min(MAX_YAW, -K_YAW * ex))
                vz = max(-MAX_UP, min(MAX_UP, -K_UP * ey))

                # 硬底线：太近就停 + 轻退
                if d_now >= D_FLOOR_PX:
                    v_fwd = -V_MIN_MS * 0.6
                    warn = f"TOO CLOSE  d_px={d_now:.0f}"

                # ★ 手动时不发任何自动指令；手动速度在循环末尾统一发送，
                #   这样在 PAUSE / TRACK / HANDOFF 下也能手动飞（原来只有
                #   APPROACH/RECAPTURE 才有 latch 重发，其它状态只发一包）。
                if not manual and now - last_sent > 0.1:
                    last_sent = now
                    if live:
                        link.send_vel(v_fwd, 0.0, vz, yr)
                    if now - last_print[0] > 0.5:
                        last_print[0] = now
                        tag = "DRY " if not live else ""
                        print(f"    {tag}cmd fwd={v_fwd:+.2f} vz={vz:+.2f} "
                              f"yr={yr:+.2f} | d_px={d_now:5.1f} "
                              f"dist={dist_m:5.1f}m score={score:.2f}")

                text(vis, f"{d_now:.1f}px  {dist_m:.1f}m  v={v_fwd:.1f}",
                     (12, H - 120), 0.65, (120, 230, 120), 2)

                if d_now >= D_HANDOFF_PX:
                    if live:
                        link.send_vel(0, 0, 0, 0)
                    enter("HANDOFF", f"d_px={d_now:.1f} >= {D_HANDOFF_PX:.1f}")
                    rec = {"ts": datetime.now().astimezone().isoformat(timespec="seconds"),
                           "state": "HANDOFF",
                           "target": {"u": cen[0], "v": cen[1], "d_px": d_now,
                                      "dist_est_m": dist_m},
                           "camera": {"fx": _state["FX"], "fy": _state["FY"],
                                      "cx": _state["CX"], "cy": _state["CY"],
                                      "width": W, "height": H,
                                      "calib": CALIB_DATE},
                           "marker_cm": MARKER_REAL_M * 100,
                           "vlm": vlm_log or None,
                           "transitions": trans}
                    _now = datetime.now()
                    # ★ 带日期：原来只有时分秒，跨天会覆盖前一天的文件
                    _tag = RUN_TAG[0] or _now.strftime("%Y%m%d_%H%M%S")
                    out = ROOT / "eval" / "demo_out" / f"handoff_{_tag}.json"
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_text(json.dumps(rec, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
                    print(f"  ★ 已交接，JSON: {out}")
                    snap = out.with_suffix(".jpg")
                    try:
                        # ★ 必须用 imwrite_unicode：cv2.imwrite 遇到中文路径
                        #   会**静默失败**（本次现场就丢了交接快照）
                        from mcpv.cv import imwrite_unicode
                        imwrite_unicode(snap, vis, quality=95)
                        print(f"  ★ 交接快照: {snap}")
                    except Exception as e:
                        print(f"  ⚠ 快照保存失败 {type(e).__name__}: {e}")

            # ★ VLM 结果可能比人工确认晚到：做一次事后核对
            if (vlm_state["done"] and not vlm_checked[0]
                    and confirm_pos[0] is not None):
                vlm_checked[0] = True
                pk = vlm_state.get("pick")
                if pk is not None:
                    dx = math.hypot(pk["cx"] - confirm_pos[0][0],
                                    pk["cy"] - confirm_pos[0][1])
                    vlm_log.update({"pick_d_px": round(pk["d"], 1),
                                    "pick_uv": [round(pk["cx"], 1),
                                                round(pk["cy"], 1)],
                                    "raw": (vlm_state.get("raw") or "")[:60],
                                    "elapsed_s": round(vlm_state.get("dt") or 0, 1),
                                    "dist_to_confirmed_px": round(dx, 1)})
                    if dx > 60:
                        print(f"  ⚠ VLM 选的是另一个候选（相距 {dx:.0f}px）。"
                              f"以人工确认为准；若不对请重新点选")

            # ★ 区分"实测"和"预测"：
            #   实测 = 绿色十字；预测 = 橙色菱形 + PRED，并连一条线到最后实测点。
            #   现场录像里预测位置飘到墙上、离真实红点 250 px，操作者以为是跟踪错了。
            if cen:
                gx, gy = int(cen[0]), int(cen[1])
                if measured:
                    cv2.drawMarker(vis, (gx, gy), (80, 240, 90),
                                   cv2.MARKER_CROSS, 24, 2)
                else:
                    cv2.drawMarker(vis, (gx, gy), (0, 165, 255),
                                   cv2.MARKER_DIAMOND, 26, 2)
                    text(vis, "PRED", (gx - 20, gy - 20), 0.55, (0, 165, 255), 2)
                    lg = getattr(trk, "last_good", None)
                    if lg is not None:
                        cv2.line(vis, (int(lg[0]), int(lg[1])), (gx, gy),
                                 (0, 165, 255), 1, cv2.LINE_AA)
                        cv2.circle(vis, (int(lg[0]), int(lg[1])), 10,
                                   (120, 120, 120), 1, cv2.LINE_AA)

        # ── 顶部状态 ────────────────────────────────
        cv2.rectangle(vis, (0, 0), (W, 42), (28, 28, 28), -1)
        scol = {"ACQUIRE": (60, 220, 255), "TRACK": (80, 240, 90),
                "APPROACH": (80, 240, 90), "RECAPTURE": (60, 200, 255),
                "HANDOFF": (0, 140, 255), "PAUSE": (60, 60, 240),
                "MANUAL": (60, 60, 240), "BLIND": (60, 200, 255),
                "STEP": (0, 200, 255),
                "SCAN": (60, 200, 255)}.get(state, (235, 235, 235))
        text(vis, f"STATE: {state}", (12, 30), 0.85, scol, 2)
        if state in ("TRACK", "APPROACH", "RECAPTURE"):
            try:
                tag2 = "实测" if measured else "预测(光流外推)"
            except NameError:
                tag2 = ""
            if tag2:
                col2 = (80, 240, 90) if tag2 == "实测" else (0, 165, 255)
                text(vis, tag2, (250, 30), 0.75, col2, 2)
        vlm_tag = ("  VLM:分析中…" if (vlm_started[0] and not vlm_state["done"])
                   else ("  VLM:已指认" if hint else ""))
        text(vis, f"{W}x{H}  {fps_ema:.1f}fps  cand={len(cands)}"
                  f"  cx={_state['CX']:.0f}" + vlm_tag,
             (W - 420, 30), 0.55, (200, 200, 200), 1)
        if warn:
            text(vis, warn, (12, H - 148), 0.7, (60, 60, 240), 2)
        if trans and trans[-1]["to"] == "HANDOFF":
            text(vis, "HANDOFF DONE — 交给破窗模块", (12, H - 170), 0.8,
                 (0, 160, 255), 2)

        # ★ 显示缩放：窗口太大超出屏幕，鼠标点不准
        sc = min(1.0, DISP_MAX_W / float(W))
        disp_scale[0] = sc
        show = vis if sc >= 0.999 else cv2.resize(
            vis, (int(W * sc), int(H * sc)), interpolation=cv2.INTER_AREA)
        if not a.headless:
            cv2.imshow("breach", show)
        if writer:
            writer.write(vis)

        # ── 按键 ────────────────────────────────────
        kb = 255 if a.headless else (cv2.waitKey(1) & 0xFF)
        if kb in (ord("x"), 27):
            print("\n★ X：停发设定点，飞机保持悬停")
            if live:
                link.send(type="exit")
            break
        elif kb == ord("l"):
            if now - land_t[0] < 3.0:
                print("★ L 确认：降落")
                if live:
                    link.send(type="land")
                break
            land_t[0] = now
            print("  再按一次 L 确认降落（3 秒内）")
        elif kb in (13, 10) and state in ("ACQUIRE", "PAUSE"):
            do_confirm()
        elif kb in (ord("w"), ord("a"), ord("s"), ord("d"),
                    ord("r"), ord("f"), ord("q"), ord("e")):
            # ★ 手动接管**不改变状态**：跟踪继续，只是不发自动指令。
            #   原来会 enter("MANUAL")，导致 trk 停止更新、模板过期，
            #   按 T 回来时已经跟丢。
            VEL = {ord("w"): (0.5, 0, 0, 0), ord("s"): (-0.5, 0, 0, 0),
                   ord("a"): (0, 0.5, 0, 0), ord("d"): (0, -0.5, 0, 0),
                   ord("r"): (0, 0, 0.4, 0), ord("f"): (0, 0, -0.4, 0),
                   ord("q"): (0, 0, 0, 0.5), ord("e"): (0, 0, 0, -0.5)}
            manual_vel[0] = VEL[kb]
            manual_until[0] = now + 0.45
            if not manual:
                manual = True
                print("  ★ 手动接管（跟踪仍在后台跑）→ 按 T 交回自动")
            if live:
                link.send_vel(*VEL[kb])
        elif kb == ord("t"):
            if manual:
                manual = False
                print("  ★ T：交回自动接近")
            elif state == "PAUSE":
                sel["cand"] = None       # ★ 清旧选择，必须重新点
                enter("ACQUIRE", "T：重新选目标（请重新单击红点）")
            else:
                print("  （当前不是手动接管状态，T 无效）")
        elif kb in (ord("+"), ord("=")):
            step_len_adj(0, +1.0)
        elif kb in (ord("-"), ord("_")):
            step_len_adj(0, -1.0)
        elif kb == ord("b") and state in ("ACQUIRE", "PAUSE") and blind["pix"]:
            bp = blind["pix"]
            trk.init(frame, bp, half_size=28, d_px=None)
            t_track[0] = time.time()
            miss[0] = 0
            good_n[0] = 0
            d_hist.clear()
            blind["t0"] = time.time()
            blind["hit_n"] = 0
            enter("BLIND", f"连续盲进 ({bp[0]:.0f},{bp[1]:.0f})")
        elif kb == ord("c") and state in ("TRACK", "APPROACH",
                                          "RECAPTURE", "PAUSE", "BLIND"):
            # ★ C = 放弃当前目标，回 ACQUIRE 重新选
            #   跟踪中点框是无效的（鼠标只在 ACQUIRE/PAUSE 生效），
            #   所以需要这个"释放目标"的入口。
            trk.reset()
            sel["cand"] = None
            blind["pix"] = None
            manual = False
            miss[0] = 0
            good_n[0] = 0
            d_hist.clear()
            step["on"] = False
            if live:
                link.send_vel(0, 0, 0, 0)
            enter("ACQUIRE", "C：放弃当前目标，请重新点选")
        elif kb == ord("g") and state in ("ACQUIRE", "PAUSE"):
            scan["on"] = True
            scan["t0"] = now
            print(f"  ★ G：开始 ±{SCAN_DEG:.0f}° 慢扫（找不到红点时用）")
        elif kb == 32:
            if live:
                link.send_vel(0, 0, 0, 0)
            print("  ⏹ 急停" + ("" if live else "（dryrun：仅记录）"))

        if a.headless and a.seconds > 0 and (time.time() - t0) > a.seconds:
            print(f"\n[headless] 跑满 {a.seconds}s，退出")
            break

        # ══ 世界系安全包络（借自 S14：水平半径 + 高度带，相对起飞点）══
        st = link.status
        if st and st.get("pos"):
            p_now = st["pos"]
            if origin[0] is None:
                origin[0] = tuple(p_now)
            ox, oy, oz = origin[0]
            r_xy = math.hypot(p_now[0] - ox, p_now[1] - oy)
            dz = p_now[2] - oz
            if r_xy > ENV_XY_MAX_M:
                warn = f"ENVELOPE 水平 {r_xy:.1f}m > {ENV_XY_MAX_M:.0f}m"
                if live:
                    link.send_vel(0, 0, 0, 0)
            elif not (ENV_Z_MIN_M <= dz <= ENV_Z_MAX_M):
                warn = f"ENVELOPE 高度 {dz:+.1f}m 超出 [{ENV_Z_MIN_M:.1f}, {ENV_Z_MAX_M:.1f}]"
                if live:
                    link.send_vel(0, 0, 0, 0)
            if st.get("volt") and st["volt"] < BATT_WARN_V:
                warn = (warn + "  " if warn else "") + \
                    f"电量低 {st['volt']:.1f}V"

        # ══ G 键扫视（借自 S14 的 ±30° 慢扫）═══════════════════
        if scan["on"]:
            el_s = now - scan["t0"]
            span = math.radians(SCAN_DEG) / SCAN_YAW_RATE   # 单边耗时
            # 相位: 0→+span 正转, span→3span 反转, 3span→4span 回中
            if el_s < span:
                yr_s = +SCAN_YAW_RATE
            elif el_s < 3 * span:
                yr_s = -SCAN_YAW_RATE
            elif el_s < 4 * span:
                yr_s = +SCAN_YAW_RATE
            else:
                scan["on"] = False
                yr_s = 0.0
                print("  扫视结束")
            if scan["on"]:
                if live and now - last_sent > 0.1:
                    last_sent = now
                    link.send_vel(0, 0, 0, yr_s)
                text(vis, f"SCAN ±{SCAN_DEG:.0f}°  {el_s:.1f}s",
                     (12, H - 200), 0.8, (60, 200, 255), 2)

        # ══ 手动速度统一发送（与状态无关）══════════════════════
        #   任何状态下按 WASD/RF/QE 都能持续飞，0.45 s latch 模拟"按住"。
        if manual and now - last_sent > 0.1:
            last_sent = now
            mv = manual_vel[0] if now < manual_until[0] else (0.0, 0.0, 0.0, 0.0)
            if live:
                link.send_vel(*mv)
            if now - last_print[0] > 0.5:
                last_print[0] = now
                print(f"    MANUAL vel={mv[0]:+.1f},{mv[1]:+.1f},"
                      f"{mv[2]:+.1f},{mv[3]:+.1f} | state={state}")

        # PAUSE：保持原地悬停（不动设定点会掉，见 README 红线 3）
        if state == "PAUSE" and live and not manual and now - last_sent > 0.2:
            last_sent = now
            link.send_vel(0, 0, 0, 0)

        # 到达 HANDOFF 后停住不动
        if state == "HANDOFF" and live:
            if int(now * 2) % 2 == 0:
                link.send_vel(0, 0, 0, 0)

    if writer:
        writer.release()
    if not a.headless:
        cv2.destroyAllWindows()
    if rec_path:
        print(f"\n录像已保存: {rec_path}")
    print("\n转移记录：")
    for r in trans:
        print(f"  {r['t']:6.1f}s  {r['from']:9s} -> {r['to']:9s}  {r['why']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
