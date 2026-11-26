#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""capture_pairs.py —— 飞机上采集双目标定图对（终端按键 + 浏览器实时画面）

为什么要在飞机上采
    stereo_calib.py 要 OpenCV >= 4.7（CharucoBoard），飞机是 4.2.0 跑不了；
    而 Windows 又拿不到这颗模组的 3840x1080 拼图档（见 focus_duo.py）。
    ⇒ 飞机上采 → 笔记本上 stereo_calib.py calib 离线标。
    它只认 <目录>/left/*.png + <目录>/right/<同名>.png，本工具按这个格式存。

用法（飞机上）
    python3 ~/tools/capture_pairs.py
    python3 ~/tools/capture_pairs.py --port 8090 --scale 0.5 --crop 600 --dir ~/calib_imgs

  终端里按键：
    回车   存一对
    d      删掉最后一对
    p      峰值清零（换距离/目标时用）
    q      退出

  浏览器看画面：
    http://192.168.2.20:8090/
    画面上直接写着：已存几对、左右眼清晰度、峰值百分比、测量框在哪。

拍多少
    工具最少要 10 对。**同一个距离拍 10 张也能出结果，但焦距和畸变会互相补偿**，
    45 m 上会偏。够用又不麻烦的最小组合（12~15 张）：
        0.7 m × 3 张（左/中/右各一）
        1.0 m × 3 张（左/中/右各一）
        1.5 m × 3 张（左/中/右各一）
        再补 3 张：板子倾斜（上下俯仰 / 左右偏航 / 绕光轴转一下）
"""
from __future__ import annotations

import argparse
import glob
import os
import select
import sys
import termios
import threading
import time
import tty
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

EYE_W, EYE_H = 1920, 1080
LOCK = threading.Lock()
STATE = {"jpg": None, "n": 0, "msg": ""}

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>标定采集</title>
<style>body{{background:#111;color:#ddd;font-family:monospace;margin:8px}}
img{{width:100%;max-width:1700px;border:1px solid #444}}</style></head><body>
<div>终端里按 <b>回车</b> 存一对 ｜ <b>d</b> 删最后一对 ｜ <b>p</b> 峰值清零 ｜ <b>q</b> 退出</div>
<div style="color:#8f8">把板子放进两只眼的画面里；数字都到峰值 80% 以上再按回车。</div>
<img src="/stream">
</body></html>"""


def find_device():
    cands = sorted(glob.glob("/dev/v4l/by-id/*"))
    hit = [p for p in cands if "index0" in p and "SPCA2100" not in p]
    if hit:
        return hit[0]
    hit = [p for p in cands if "SPCA2100" not in p and "index0" in p]
    return hit[0] if hit else ""


def sharpness(gray):
    g = gray.astype(np.float32)
    m = float(g.mean())
    return float(cv2.Laplacian(g, cv2.CV_32F, ksize=3).var()) / ((m + 1e-6) ** 2) * 1000.0, m


def crop_center(g, n):
    if not n:
        return g
    h, w = g.shape[:2]
    s = min(n, h, w)
    y0, x0 = (h - s) // 2, (w - s) // 2
    return g[y0:y0 + s, x0:x0 + s]


def tag(img, s, xy, col=(0, 255, 0), sc=0.8, th=2):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, sc, (0, 0, 0), th + 2)
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, sc, col, th)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.split("?")[0] == "/":
            b = PAGE.format().encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif self.path.split("?")[0] == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with LOCK:
                        jpg = STATE["jpg"]
                    if not jpg:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.08)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.expanduser("~/calib_imgs"))
    ap.add_argument("--dev", default=None)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--crop", type=int, default=600)
    ap.add_argument("--color", action="store_true", help="存彩色（默认灰度，体积减半）")
    ap.add_argument("--scale", type=float, default=0.5, help="网页预览缩放（不影响保存的原图）")
    ap.add_argument("--quality", type=int, default=75)
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--auto", type=int, default=0, help="自检：自动存 N 对后退出")
    ap.add_argument("--seconds", type=float, default=0, help="自检：跑满 N 秒后退出")
    a = ap.parse_args()

    ldir, rdir = os.path.join(a.dir, "left"), os.path.join(a.dir, "right")
    os.makedirs(ldir, exist_ok=True)
    os.makedirs(rdir, exist_ok=True)
    n0 = len(sorted(glob.glob(os.path.join(ldir, "*.png"))))
    if n0:
        print("[capture] %s 里已有 %d 对，接着编号" % (a.dir, n0))

    dev = a.dev or find_device()
    if not dev:
        sys.exit("[capture] 找不到相机：--dev 指定")
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit("[capture] 打不开 %s（focus_web / 桥 占着？先停掉）" % dev)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, EYE_W * 2)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, EYE_H)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    ok = False
    frame = None
    for _ in range(20):
        ok, frame = cap.read()
        if ok and frame is not None:
            break
    if not ok or frame is None:
        sys.exit("[capture] 读不到帧")
    Hh, Ww = frame.shape[:2]
    if Ww != EYE_W * 2:
        print("[capture] ⚠ 没拿到 3840x1080 拼图档（实际 %dx%d），标定会失败" % (Ww, Hh))

    # ---- 网页预览（独立线程）----
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", a.port), H)
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        host = os.uname().nodename if hasattr(os, "uname") else "127.0.0.1"
        print("[capture] ★ 浏览器看画面： http://%s:%d/   （建议同时开两个窗口）" % (host, a.port))
    except Exception as e:
        print("[capture] 网页预览起不来（%s），照常在终端里拍" % e)

    print("[capture] 设备 %s  出图 %dx%d（每眼 %dx%d）  存到 %s"
          % (dev, Ww, Hh, Ww // 2, Hh, a.dir))
    print("[capture] 按键：回车=存一对  d=删最后  p=峰值清零  q=退出")
    print("-" * 96)

    fd = sys.stdin.fileno()
    old = None
    interactive = sys.stdin.isatty() and not a.auto
    if interactive:
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)

    peak = {"L": 0.0, "R": 0.0}
    saved, t0, t_print = 0, time.time(), 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            L = frame[:, :Ww // 2]
            R = frame[:, Ww // 2:2 * (Ww // 2)]
            vals, lums = {}, {}
            for k, im in (("L", L), ("R", R)):
                g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
                vals[k], lums[k] = sharpness(crop_center(g, a.crop))
                peak[k] = max(peak[k], vals[k])

            # ---- 画面叠加 ----
            vis = frame.copy()
            cw = Ww // 2
            if a.crop:
                s = min(a.crop, Hh, cw)
                y0, x0 = (Hh - s) // 2, (cw - s) // 2
                for ox in (0, cw):
                    cv2.rectangle(vis, (ox + x0, y0), (ox + x0 + s, y0 + s), (0, 255, 0), 3)
            cv2.line(vis, (cw, 0), (cw, Hh), (0, 200, 255), 3)
            for k, ox in (("L", 0), ("R", cw)):
                pct = vals[k] / peak[k] * 100 if peak[k] else 100.0
                tag(vis, "%s  %6.0f  pk %6.0f  %3.0f%%" % (k, vals[k], peak[k], pct),
                    (ox + 14, 42), (0, 255, 0) if pct >= 80 else (0, 255, 255), 1.0, 2)
            tag(vis, "SAVED %d   [Enter]=save pair" % saved, (14, Hh - 20), (0, 255, 255), 1.0, 2)
            if a.scale != 1.0:
                vis = cv2.resize(vis, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
            okj, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), a.quality])
            if okj:
                with LOCK:
                    STATE["jpg"] = buf.tobytes()
                    STATE["n"] = saved

            # ---- 按键 ----
            key = None
            if interactive:
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if r:
                    key = sys.stdin.read(1)
            elif a.auto:
                key = "\r"

            if key in ("\r", "\n"):
                name = "%03d.png" % (n0 + saved)
                for eye, im in (("L", L), ("R", R)):
                    out = im if a.color else cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
                    cv2.imwrite(os.path.join(a.dir, "left" if eye == "L" else "right", name), out)
                saved += 1
                pct = min(vals["L"] / peak["L"] * 100 if peak["L"] else 100.0,
                          vals["R"] / peak["R"] * 100 if peak["R"] else 100.0)
                print("  [%03d] 已存  L=%.0f R=%.0f  (%.0f%%)%s"
                      % (n0 + saved - 1, vals["L"], vals["R"], pct,
                         "  ⚠ 偏虚，建议重拍" if pct < 80 else ""), flush=True)
                if a.auto and saved >= a.auto:
                    break
            elif key == "d":
                if saved:
                    name = "%03d.png" % (n0 + saved - 1)
                    for sub in ("left", "right"):
                        p = os.path.join(a.dir, sub, name)
                        if os.path.exists(p):
                            os.remove(p)
                    saved -= 1
                    print("  删掉 %s，本次还剩 %d 对" % (name, saved), flush=True)
            elif key == "p":
                peak.update(L=0.0, R=0.0)
                print("  峰值清零", flush=True)
            elif key == "q":
                break

            now = time.time() - t0
            if now - t_print >= 1.0:
                t_print = now
                print("  L %6.0f (%3.0f%%) ｜ R %6.0f (%3.0f%%) ｜ 本次已存 %d"
                      % (vals["L"], vals["L"] / peak["L"] * 100 if peak["L"] else 100,
                         vals["R"], vals["R"] / peak["R"] * 100 if peak["R"] else 100, saved),
                      flush=True)
            if a.seconds and now > a.seconds:
                break
    finally:
        if old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        cap.release()
        total = len(glob.glob(os.path.join(ldir, "*.png")))
        print("\n[capture] 收工。%s 共 %d 对" % (a.dir, total))
        if total < 10:
            print("[capture] ★ 少于 10 对，stereo_calib.py 会拒绝标定")
        else:
            print("[capture] 下一步（笔记本上）：")
            print("    scp -r visbot@192.168.2.20:%s ." % a.dir)
            print("    python stereo_calib.py calib --dir calib_imgs --square 28 --calib stereo_calib.yaml")
    return 0


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass
    sys.exit(main())
