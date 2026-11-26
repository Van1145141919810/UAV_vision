# -*- coding: utf-8 -*-
r"""一条命令跑完：图 → 选一个目标 → VLM 判语义 → 几何算位移 → 存 JSON。

不需要点鼠标，适合快速验证 / 汇报截图。交互版见 demo_click.py。

用法：
    python scripts/demo_scene.py                       # 用 win03，自动挑一扇窗
    python scripts/demo_scene.py data\samples\win06.jpg
    python scripts/demo_scene.py data\samples\win06.jpg --click 2100,1400
    python scripts/demo_scene.py data\samples\test3.jpg --click 880,690 --note
"""

import sys
import json
import argparse
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import cv2                                   # noqa: E402

from mcpv import config as C                 # noqa: E402
from mcpv.aim import Aimer                   # noqa: E402
from mcpv.cv import imread_unicode           # noqa: E402
from mcpv.detect import load_bgr             # noqa: E402
from mcpv.report import make_record, save_record   # noqa: E402
from mcpv.scene import describe_target       # noqa: E402
from mcpv.windows import find_windows        # noqa: E402


def console():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
        except Exception:
            pass
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass


def crop_around(img, u, v, half=200):
    h, w = img.shape[:2]
    x0, y0 = max(0, int(u - half)), max(0, int(v - half))
    x1, y1 = min(w, int(u + half)), min(h, int(v + half))
    patch = img[y0:y1, x0:x1]
    if patch.size == 0:
        patch = img
    return cv2.resize(patch, (384, 384), interpolation=cv2.INTER_LANCZOS4)


def main():
    console()
    ap = argparse.ArgumentParser()
    ap.add_argument("image", nargs="?", default=str(C.SAMPLES_DIR / "win03.jpg"))
    ap.add_argument("--click", help="原图像素坐标 u,v；不给就自动挑第一个窗框")
    ap.add_argument("--step", type=float, default=C.CLICK_STEP_M)
    ap.add_argument("--note", action="store_true", help="多问一句自由描述（慢 1 秒）")
    ap.add_argument("--no-vlm", action="store_true", help="跳过语义层（只看几何）")
    args = ap.parse_args()

    p = Path(args.image)
    bgr = load_bgr(p)
    h, w = bgr.shape[:2]
    aim = Aimer.from_config(w, h)

    # ---------- 1. 选目标 ----------
    if args.click:
        u, v = [float(x) for x in args.click.split(",")]
        sel = {"kind": "point", "point": [u, v]}
        crop = crop_around(bgr, u, v)
        print(f"[1] 手动选点 ({u:.0f},{v:.0f})")
    else:
        boxes = find_windows(bgr)
        if not boxes:
            print("[!] 没找到窗户，改用画面中心")
            u, v = w / 2, h / 2
            sel = {"kind": "point", "point": [u, v]}
            crop = crop_around(bgr, u, v)
        else:
            # 挑一个"面积中位数"的框，避免选到过大/过小的
            boxes = sorted(boxes, key=lambda b: b[2] * b[3])
            x, y, bw, bh = boxes[len(boxes) // 2]
            u, v = x + bw / 2, y + bh / 2
            sel = {"kind": "win", "bbox_px": [int(x), int(y), int(x + bw), int(y + bh)]}
            crop = crop_around(bgr, u, v, half=max(60, int(max(bw, bh) * 0.7)))
            print(f"[1] 自动挑窗 共 {len(boxes)} 个，选中 ({u:.0f},{v:.0f}) size={bw}x{bh}")

    # ---------- 2. 语义层 ----------
    if args.no_vlm:
        scene = None
        print("[2] 跳过语义层")
    else:
        t0 = time.time()
        try:
            scene = describe_target(crop, want_note=args.note)
            print(f"[2] 语义: 窗={scene['window_state']}  反光={scene['glare']}"
                  f"  ({time.time()-t0:.1f}s)")
            if scene.get("note"):
                print(f"    描述: {scene['note']}")
        except Exception as e:
            scene = None
            print(f"[2] 语义层失败（{type(e).__name__}: {e}），继续只用几何")

    # ---------- 3. 几何 ----------
    cmd = aim.command(u, v, step_m=args.step)
    print(f"[3] 位移: {cmd.describe()}")

    # ---------- 4. 存档 ----------
    rec = make_record(str(p), (w, h), sel, cmd, scene)
    out = save_record(rec)
    print(f"[4] 已存: {out}")
    print("\n----- JSON -----")
    print(json.dumps(rec, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
