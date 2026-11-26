# -*- coding: utf-8 -*-
"""grabd.py —— 常驻抓图（支持运行时切换模式）

═══════════════════════════════════════════════════════════════════
两种模式，用 /tmp/cpmv_mode 一个文件切换（笔记本按 M 键写它）
───────────────────────────────────────────────────────────────────
  hd     MJPG 3840 全分辨率    ~0.06 s/帧  边缘糊（11 px）  → ★默认，跟踪用
  fine   YUYV 3840 全分辨率    ~1.5 s/帧   边缘锐（5 px）   → 需要极致清晰时
  video  MJPG 3840 缩一半      ~0.06 s/帧  边缘更糊         → 带宽紧张时

为什么非要切换：YUYV 每帧 8 MB，USB2 传不过来，相机本身就被限到 1.5 s/帧。
手动飞时 0.6 fps 根本没法用；换 MJPG 缩到 960×540 才能到 4~5 fps。

切换代价：重开相机。MJPG 约 2 秒，YUYV 约 10 秒（首帧慢）。
          所以切回 fine 会顿一下，这是硬件限制，不是 bug。

注意：相机独占。跑之前确认文豪的桥没在跑；/dev/video35/36 是 VINS，别碰。
"""
import argparse
import os
import sys
import time

import cv2

DEV = "/dev/video33"
OUT = "/tmp/cpmv_frame.jpg"
TMP = "/tmp/cpmv_frame.new"
MODE_FILE = "/tmp/cpmv_mode"       # 内容 "fine" 或 "video"

PROFILES = {
    #         模式    宽     高    缩放
    "fine":  ("yuyv", 3840, 1080, 1.0),   # 最锐，~1.5 s/帧（实测有时 7 s）
    "video": ("mjpg", 3840, 1080, 0.5),   # 960x540，~6 fps
    # ★ 新增：MJPG 全分辨率 —— 1920x1080 且 ~16 fps。
    #   MJPG 每帧 0.06 s（README 2026-11-04 实测），
    #   边缘过渡 11 px（YUYV 是 5 px），但红点有 20+ px，
    #   这点糊完全不影响检测，却换来 16 倍的帧率。
    "hd":    ("mjpg", 3840, 1080, 1.0),
}
DEFAULT_MODE = "hd"


def open_cam(mode, w, h):
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"打不开 {DEV}（被占用？）")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(
        *("YUYV" if mode == "yuyv" else "MJPG")))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


def read_mode():
    try:
        with open(MODE_FILE) as f:
            v = f.read().strip()
        return v if v in PROFILES else DEFAULT_MODE
    except Exception:
        return "fine"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=list(PROFILES), default=None,
                    help="不给就用 /tmp/cpmv_mode，再没有就 fine")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()

    cur = args.mode or read_mode()
    fourcc, w, h, scale = PROFILES[cur]
    print(f"[grabd] 打开 {DEV} {cur}: {fourcc} {w}x{h} scale={scale} …（首帧可能要 10 秒）")
    cap = open_cam(fourcc, w, h)

    n = 0
    t_start = time.time()
    while True:
        # ---- 检查要不要切模式 ----
        want = args.mode or read_mode()
        if want != cur:
            print(f"[grabd] 切换 {cur} → {want}")
            cap.release()
            cur = want
            fourcc, w, h, scale = PROFILES[cur]
            time.sleep(0.3)
            cap = open_cam(fourcc, w, h)
            n = 0
            t_start = time.time()

        t0 = time.time()
        ok, frame = cap.read()
        if not ok:
            print("[grabd] 读帧失败，重开相机", file=sys.stderr)
            cap.release()
            time.sleep(1.0)
            cap = open_cam(fourcc, w, h)
            continue

        hh, ww = frame.shape[:2]
        if ww >= 2 * hh:                       # 双目并排 → 左目
            frame = frame[:, : ww // 2]

        if scale != 1.0:
            frame = cv2.resize(frame, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA)

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if ok:
            with open(TMP, "wb") as f:
                f.write(buf.tobytes())
            os.replace(TMP, args.out)
            with open(args.out + ".stamp", "w") as f:
                f.write("%.3f" % time.time())

        n += 1
        if n == 1 or n % 20 == 0:
            fps = n / (time.time() - t_start)
            print(f"[grabd] {cur} 第{n}帧 {frame.shape[1]}x{frame.shape[0]} "
                  f"每帧{time.time()-t0:.2f}s 平均{fps:.2f}fps")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[grabd] 停")
