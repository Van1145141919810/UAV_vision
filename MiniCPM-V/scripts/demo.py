# -*- coding: utf-8 -*-
"""一键演示：把 data/samples 里的图全跑一遍，打印表格、存标注图。

在 VS Code 里按 Ctrl+Shift+B（或 F5）就会跑这个。
也可以双击项目根目录的 run_demo.cmd。
"""

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mcpv import config as C            # noqa: E402
from mcpv.detect import detect, load_bgr, save_all   # noqa: E402

# 人工核对过的真值（新图不在表里就按"未知"处理，不计入准确率）
TRUTH = {
    "test1.jpg": False,   # 园林古塔，无红点
    "test2.jpg": True,    # 办公室，白板纸上红点 53 px
    "test3.jpg": True,    # 白板特写，红点 110 px
    "neg1.jpg": False,    # 白墙
    "neg2.jpg": False,    # 桌面
    "neg3.jpg": False,    # 夜景楼宇
    "neg4.jpg": False,    # 楼梯
    "neg5.jpg": False,    # 天花板
}

EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def main() -> int:
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

    files = sorted(f for f in C.SAMPLES_DIR.iterdir() if f.suffix.lower() in EXTS and f.name in TRUTH)
    if not files:
        print(f"[x] {C.SAMPLES_DIR} 里没有图片")
        return 1

    print("=" * 78)
    print(f"MiniCPM-V 红点检测演示   共 {len(files)} 张图")
    print(f"Python: {sys.executable}")          # 一眼看出跑的是哪个环境
    print(f"样本目录: {C.SAMPLES_DIR}")
    print(f"结果目录: {C.RESULTS_DIR}")
    print("=" * 78)

    t_all = time.time()
    rows, correct, labeled = [], 0, 0

    for f in files:
        bgr = load_bgr(f)
        res = detect(f)
        save_all(bgr, res)

        truth = TRUTH.get(f.name)
        mark = "-"
        if truth is not None:
            labeled += 1
            ok = (res.has_target == truth)
            correct += ok
            mark = "✓" if ok else "✗"
        rows.append((f.name, truth, res, mark))
        print(f"  {f.name:12s} 候选{res.n_candidates:2d} 过CV{res.n_passed_cv:2d} "
              f"VLM{res.vlm_calls}次 {res.elapsed_s:5.1f}s  "
              f"⇒ {'有' if res.has_target else '无'}  {mark}")

    print("-" * 78)
    if labeled:
        print(f"准确率: {correct}/{labeled} = {correct / labeled * 100:.0f}%")
    print(f"总耗时: {time.time() - t_all:.1f}s")
    print(f"标注图已存到: {C.RESULTS_DIR}")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
