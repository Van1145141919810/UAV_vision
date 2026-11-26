# -*- coding: utf-8 -*-
"""probe_stream.py —— 连上飞机后先跑这个：测视频流的真实分辨率/帧率/延迟

═══════════════════════════════════════════════════════════════════
为什么必须先测
───────────────────────────────────────────────────────────────────
红点在画面里的像素直径 = f × 红圆直径 ÷ 距离，而 **f 随分辨率等比缩放**。
所以"视频流输出多少分辨率"直接决定：
  · 红圆要做多大（20 cm 还是 26 cm）
  · 交接阈值是多少像素
  · 50 m 处还能不能检出

这个脚本把三件事测出来：
  ① TCP vstream 的实际分辨率、帧率、单帧 JPEG 大小
  ② 若 vstream 不通，scp 拉单帧的耗时（兜底方案）
  ③ 按实测分辨率算出 50/35/20/10/6.5 m 各自的红点像素直径
"""
import socket
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

DRONE = "visbot@192.168.2.20"
DRONE_IP = "192.168.2.20"
VIDEO_PORT = 8890
SSH_KEY = str(Path.home() / ".ssh" / "dsh_drone")
# 与 breach_run.py 同一套内参（来自 S14 stereo_calib.yaml 2026-11-04）
FX_PX_1920 = 1566.149
FY_PX_1920 = 1573.382
CX_PX_1920 = 932.277
CY_PX_1920 = 517.261

MARKERS = [("A4 20 cm", 0.20), ("A3 26 cm", 0.26), ("旧 15 cm", 0.15)]
DISTS = [50.0, 35.0, 20.0, 10.0, 6.5, 3.0]


def recv_n(sock, n):
    buf = b""
    while len(buf) < n:
        b = sock.recv(n - len(buf))
        if not b:
            raise ConnectionError("closed")
        buf += b
    return buf


def probe_tcp(timeout=8.0, nframes=20):
    print(f"\n① TCP vstream  {DRONE_IP}:{VIDEO_PORT}")
    try:
        s = socket.create_connection((DRONE_IP, VIDEO_PORT), timeout=timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception as e:
        print(f"  ✗ 连不上：{e}")
        return None
    sizes, dts, imgs = [], [], 0
    t_prev = time.time()
    for i in range(nframes):
        try:
            ln = struct.unpack(">I", recv_n(s, 4))[0]
            if ln > 8_000_000:
                print(f"  ✗ 帧长度异常 {ln}")
                break
            data = recv_n(s, ln)
        except Exception as e:
            print(f"  ✗ 收帧失败：{e}")
            break
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        now = time.time()
        if imgs > 0:
            dts.append(now - t_prev)
        t_prev = now
        imgs += 1
        sizes.append(ln)
    s.close()
    if imgs == 0:
        print("  ✗ 没收到有效帧")
        return None
    h, w = img.shape[:2]
    fps = 1.0 / (sum(dts) / len(dts)) if dts else 0.0
    print(f"  ✓ 分辨率 {w}x{h}   帧率 {fps:.1f} fps   "
          f"单帧 {np.mean(sizes)/1024:.0f} KB   （{imgs} 帧）")
    return {"w": w, "h": h, "fps": fps, "kb": float(np.mean(sizes) / 1024)}


def probe_scp():
    print("\n② scp 拉单帧（兜底方案）")
    local = Path.home() / "AppData/Local/Temp" / "probe_frame.jpg"
    ts = []
    for i in range(3):
        t0 = time.time()
        r = subprocess.run(["scp", "-i", SSH_KEY,
                            "-o", "StrictHostKeyChecking=no",
                            f"{DRONE}:~/cpmvbyhuang/frame.jpg", str(local)],
                           capture_output=True, timeout=30)
        dt = time.time() - t0
        if r.returncode != 0:
            print(f"  ✗ scp 失败：{r.stderr.decode()[:80]}")
            return None
        ts.append(dt)
    img = cv2.imdecode(np.fromfile(str(local), dtype=np.uint8), cv2.IMREAD_COLOR)
    h, w = img.shape[:2] if img is not None else (0, 0)
    print(f"  ✓ {w}x{h}   单帧耗时 {np.mean(ts):.2f} s "
          f"（≈{1/np.mean(ts):.1f} fps）")
    return {"w": w, "h": h, "fps": 1.0 / np.mean(ts)}


def report(info):
    if not info:
        print("\n没有可用视频流，无法换算。")
        return
    w = info["w"]
    h = info["h"]
    f = FX_PX_1920 * w / 1920.0
    cx = CX_PX_1920 * w / 1920.0
    cy = CY_PX_1920 * h / 1080.0
    print(f"\n③ 按 {w}x{h} 换算：fx={f:.1f} fy={FY_PX_1920*h/1080.0:.1f} "
          f"cx={cx:.1f} cy={cy:.1f}（主点不在画面中心）")
    print(f"{'红圆':<12}" + "".join(f"{d:>8.0f}m" for d in DISTS))
    print("-" * 64)
    for name, real in MARKERS:
        row = "".join(f"{f*real/d:>9.1f}" for d in DISTS)
        print(f"{name:<12}{row}")
    print("\n判读（像素直径）：")
    print("  ≥9  可锁定    ≥6  可检出    3~5  临界     <3  基本没戏")
    handoff = 6.5
    for name, real in MARKERS:
        print(f"  {name:<10} 交接点 {handoff} m → d_px {f*real/handoff:5.1f}")


if __name__ == "__main__":
    print("=" * 64)
    print("  视频流探测（先跑这个，再决定红圆尺寸）")
    print("=" * 64)
    tcp = probe_tcp()
    if tcp is None:
        tcp = probe_scp()
    report(tcp)
