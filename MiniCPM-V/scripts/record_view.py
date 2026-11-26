# -*- coding: utf-8 -*-
"""record_view.py —— 在**笔记本**上录「识别画面」，用于交差放映和事后分析

为什么要它（和 `录像工具/录视频.cmd` 分工不同）
──────────────────────────────────────────────────────────────
    录像工具：连到飞机，录**相机原始话题**。画质最好，但
              ① 编码吃飞机 CPU（文档自己写了"别在飞行时录 1080p"）
              ② 录到的画面上没有候选编号 / 建议 / 阶段 —— 上级看不出我们做了什么
    本脚本：  在笔记本上按固定帧率拉 `:8091` 或 `:8093` 的图，压成 MP4，
              并把「时间 / 阶段 / 候选数 / 距离」烧进角落。零飞机负担，
              视频时长 == 飞行时长（缺帧用重复帧补齐，不会跳秒）。

用法
    python scripts/record_view.py                     # 录到 Ctrl+C，存 D:\\Desktop\\无人系统\\视频
    python scripts/record_view.py --seconds 60         # 录 60 秒
    python scripts/record_view.py --src 8091           # 录原样画面（不开顾问时用这个）
    python scripts/record_view.py --src 8093 --state   # 录带建议的画面 + 叠阶段/距离

产出
    <out>/view_20261121_141530.mp4    视频
    <out>/view_20261121_141530.json   sidecar：起止时间 / 帧数 / 实际帧率 / 来源（分析用）
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
try:
    sys.stdout.reconfigure(errors="replace")   # 控制台是 GBK，遇到生僻字符不要崩
except Exception:
    pass
DEFAULT_OUT = r"D:\Desktop\无人系统\视频"

_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _get(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def _state_text(state_url, cache=None):
    """从 /api/state 取几个关键字段，压成一行**纯 ASCII**。

    为什么必须 ASCII：这里用 cv2.putText 烧字（每帧都要画，用 PIL 太慢），
    而 Hershey 字体没有中文字形 —— 写中文会变成方块/乱码，交差时很难看。
    为什么要 cache：视频 10 fps，每帧都请求就是 10 次/s 的 HTTP；
    实测会把录制拖慢到 1/5 速。缓存 0.5 s 足够（协议里阶段变化以秒计）。
    """
    now = time.time()
    if cache is not None and cache.get("t") and now - cache["t"] < 0.5:
        return cache.get("v")
    try:
        st = json.loads(_get(state_url).decode("utf-8"))
    except Exception:
        if cache is not None:
            cache["t"] = now          # 失败也记时间，别每帧重试
        return None
    g = st.get("gate") or {}
    n = len(g.get("cands") or [])
    stage = g.get("stage") or g.get("seg") or "?"
    z = g.get("dist") or (g.get("gate_used") or {}).get("dist")
    txt = "SEG %s  CAND %d" % (str(stage), n)
    if isinstance(z, (int, float)):
        txt += "  %.1fm" % z
    if cache is not None:
        cache["t"], cache["v"] = now, txt
    return txt


def _open_writer(path_base, fps, size, want="auto"):
    """want: auto = 先 avc1(H.264) 再 mp4v；avc1 / mp4v / mjpg(.avi，最稳但文件大)。

    实测本机 avc1 有时能出 h264、有时退到 mp4v（缺 openh264 dll）。
    两种都能录、都能读回；mp4v 在老播放器上可能打不开，所以把选择留给你。
    """
    if want == "mjpg":
        p = "%s.avi" % path_base
        w = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*"MJPG"), fps, size)
        return (w, p, "mjpg") if w.isOpened() else (None, None, None)
    order = [("avc1", 2), ("mp4v", 1)] if want == "auto" else [(want, 2)]
    for cc, tries in order:
        for _ in range(tries):
            p = "%s.mp4" % path_base
            w = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*cc), fps, size)
            if w.isOpened():
                return w, p, cc
            w.release()
    return None, None, None


def _put(img, text, xy, scale=0.6, col=(255, 255, 255), th=1):
    cv2.putText(img, text, xy, _FONT, scale, (0, 0, 0), th + 2)
    cv2.putText(img, text, xy, _FONT, scale, col, th)


def _corner_box(img, lines, scale=0.55, pad=6):
    """右下角半透明黑底 + 右对齐两行字。

    为什么放右下：他系统自己的叠加占了左上（阶段）、右上（分辨率/帧率）、
    左下（像素/距离），只剩右下是空的。
    """
    h, w = img.shape[:2]
    texts = [t for t, _ in lines]
    sizes = [cv2.getTextSize(t, _FONT, scale, 1)[0] for t in texts]
    bw = max(s[0] for s in sizes) + pad * 2
    lh = max(s[1] for s in sizes) + 4
    bh = lh * len(lines) + pad * 2
    x0, y0 = w - bw - 10, h - bh - 10
    roi = img[y0:y0 + bh, x0:x0 + bw]
    shade = np.zeros_like(roi)
    cv2.addWeighted(shade, 0.55, roi, 0.45, 0, roi)   # 半透明压暗，不挡画面
    for i, (t, col) in enumerate(lines):
        tw = sizes[i][0]
        cv2.putText(img, t, (x0 + bw - pad - tw, y0 + pad + lh * (i + 1) - 4),
                    _FONT, scale, (0, 0, 0), 3)
        cv2.putText(img, t, (x0 + bw - pad - tw, y0 + pad + lh * (i + 1) - 4),
                    _FONT, scale, col, 1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="8093",
                    help="录哪个画面：8093(带建议) / 8091(原样) / 或完整 URL")
    ap.add_argument("--state", default="http://127.0.0.1:8080/api/state",
                    help="叠阶段/候选数用的状态接口（连不上就不叠）")
    ap.add_argument("--no-state", action="store_true", help="不叠状态")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = 录到 Ctrl+C")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--max-width", type=int, default=0, help="缩到多宽（0=原样）")
    ap.add_argument("--codec", choices=["auto", "avc1", "mp4v", "mjpg"], default="auto",
                    help="auto=先 H.264 再退 mp4v；mjpg 写成 .avi，任何播放器都能开")
    a = ap.parse_args()

    src = a.src if a.src.startswith("http") else "http://127.0.0.1:%s/snapshot.jpg" % a.src
    out_dir = Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = str(out_dir / ("view_%s" % stamp))

    print("录的是: %s" % src)
    print("存的到: %s" % base)

    # 先拿一帧定尺寸；一个口没起就自动换另一个（8093 <-> 8091），少一次踩空
    def _grab(url, tries=6):
        for _ in range(tries):
            try:
                b = _get(url)
                im = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
                if im is not None:
                    return im
            except Exception:
                pass
            time.sleep(1.0)
        return None

    img0 = _grab(src)
    if img0 is None and not a.src.startswith("http"):
        alt = "8091" if a.src != "8091" else "8093"
        alt_url = "http://127.0.0.1:%s/snapshot.jpg" % alt
        print("  %s 没画面，自动试 %s ..." % (src, alt_url))
        img0 = _grab(alt_url, tries=4)
        if img0 is not None:
            src, a.src = alt_url, alt
            print("  改用 %s" % src)
    if img0 is None:
        print("[X] 拉不到画面：%s" % src)
        print("    确认 8093（顾问）或 8091（地面站）在跑；也可以先开 起飞.cmd -> 2 准备飞行。")
        return 2

    if a.max_width and img0.shape[1] > a.max_width:
        img0 = cv2.resize(img0, (a.max_width,
                                 int(round(img0.shape[0] * a.max_width / float(img0.shape[1])))))
    h, w = img0.shape[:2]
    writer, path, codec = _open_writer(base, a.fps, (w, h), a.codec)
    if writer is None:
        print("[X] 打不开编码器（avc1 / mp4v 都不行）")
        return 3
    print("  尺寸 %dx%d  帧率 %.1f  编码 %s" % (w, h, a.fps, codec))

    t_start = time.time()
    n = 0
    got = 0
    last = img0
    last_txt = None
    state_bad = 0
    scache = {}
    try:
        while True:
            if a.seconds and (time.time() - t_start) >= a.seconds:
                break
            try:
                b = _get(src)
                img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    got += 1
                    last = img
            except Exception:
                pass
            frame = last.copy()
            if frame.shape[1] != w or frame.shape[0] != h:
                frame = cv2.resize(frame, (w, h))

            # 右下角角标：时间 + 已录时长 + 来源端口；第二行是阶段/候选数/距离
            el = time.time() - t_start
            port = src.rstrip("/").split("/")[-2].split(":")[-1] if ":" in src else "?"
            lines = [("%s  +%05.1fs  :%s" % (time.strftime("%H:%M:%S"), el, port),
                      (200, 255, 200))]
            if not a.no_state:
                txt = _state_text(a.state, scache)
                if txt:
                    last_txt = txt
                elif txt is None:
                    state_bad += 1
                if last_txt:
                    lines.append((last_txt, (255, 220, 120)))
            _corner_box(frame, lines)

            writer.write(frame)
            n += 1
            if n % int(max(1, a.fps)) == 0:
                print("  已录 %5.1fs  %d 帧  (拉到新图 %d 次)" % (el, n, got), flush=True)
            # 按固定帧率节奏写，保证视频时长 == 实际时长
            time.sleep(max(0.0, (t_start + n / a.fps) - time.time()))
    except KeyboardInterrupt:
        print("\n  手动停止")
    finally:
        writer.release()

    dur = n / a.fps if a.fps else 0
    size = os.path.getsize(path) if os.path.exists(path) else 0

    # 自检：把刚写的文件重新打开读一遍（和录像工具同一套判据）
    cap = cv2.VideoCapture(path)
    vn = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    vf = float(cap.get(cv2.CAP_PROP_FPS) or 0)
    cap.release()
    ok = vn > 0 and abs(vn - n) <= 2

    print()
    print("=" * 60)
    print("  文件   : %s" % path)
    print("  时长   : %.1f s（写 %d 帧 @ %.1f fps）" % (dur, n, a.fps))
    print("  大小   : %.1f MB" % (size / 1e6))
    print("  实际拉到新图: %d 次（≈%.1f Hz，其余为重复帧）"
          % (got, got / max(dur, 1e-6)))
    if not a.no_state and state_bad > int(n * 0.8):
        print("  注意   : 状态接口基本不可用，角标里没有阶段/候选数")
    if codec == "mp4v":
        print("  播放   : 编码 mp4v（老播放器可能打不开）—— 用 VLC / PotPlayer 打开；")
        print("           要发给上级，重录时加 --codec mjpg（存 .avi，任何播放器都能开）")
    print("  自检   : %s（重开读回 %d 帧 / %.1f fps）"
          % ("OK" if ok else "帧数对不上", vn, vf))
    print("=" * 60)

    with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as f:
        json.dump({"src": src, "start": time.strftime("%Y-%m-%d %H:%M:%S",
                                                      time.localtime(t_start)),
                   "end": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "seconds": round(dur, 2), "frames": n, "fps": a.fps,
                   "pulled_new": got, "size_bytes": size, "codec": codec,
                   "size": [w, h]}, f, ensure_ascii=False, indent=1)
    print("  sidecar: %s" % (os.path.splitext(path)[0] + ".json"))
    return 0 if ok else 4


if __name__ == "__main__":
    sys.exit(main())
