# -*- coding: utf-8 -*-
"""命令行入口：一张图 / 一个目录，跑完整链路并输出结果。

三种用法（对应三种"我要看什么"）:
    1) 人看：      python -m mcpv.cli 图片路径
                   —— 打印候选表、判定、耗时
    2) 给程序看：  python -m mcpv.cli 图片路径 --json
                   —— 只吐 JSON（交给 Jev / 下游脚本）
    3) 存档：      python -m mcpv.cli 图片路径 --save
                   —— 额外写 data/results/<名字>.json 和 <名字>_annotated.jpg

设计约束（踩过的坑，别改回去）:
    · **一张图只能 detect 一次**：最早的版本在 --json 分支之外又调了一次
      run_one()，导致同一张图跑两遍模型（白花一次 6 秒）。现在统一走 _one()。
    · CV-only 模式（--no-vlm）下 has_target 的语义是「有候选就算有」，
      它不是最终判断，只用于消融对比 / 模型没起时的兜底。
"""

import argparse
import json
import sys
from pathlib import Path

from .detect import detect, load_bgr, save_all
from .vlm import VLMError

EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def fix_console():
    """Windows 控制台默认 GBK，直接打印中文会抛 UnicodeEncodeError。"""
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


def _print_human(res):
    """人读的版本：候选明细 + 结论。"""
    print(f"\n{res.image}  {res.image_size[0]}x{res.image_size[1]}")
    print(f"  候选 {res.n_candidates} 个 → 通过筛选 {res.n_passed_cv} 个"
          f"  (VLM 调用 {res.vlm_calls} 次, {res.elapsed_s:.1f}s)")
    for c in res.candidates:
        v = c.vlm_verdict or "-"
        print(f"    ({c.cx:4d},{c.cy:4d}) d={c.diameter_px:5.1f}px "
              f"circ={c.circularity:.2f} score={c.score:6.1f} "
              f"{'浅底' if c.white_ok else '    '}  VLM={v}")
    print(f"  ⇒ {'有目标' if res.has_target else '无目标'}")


def _one(p: Path, use_vlm: bool, save: bool, quiet: bool, as_json: bool) -> bool:
    """处理一张图：只检测一次，然后按需要输出。返回是否判为目标。"""
    bgr = load_bgr(p)
    try:
        res = detect(p, use_vlm=use_vlm)
    except VLMError as e:
        # 模型没起的时候不许崩，给一句能看懂的话
        print(f"[x] 模型调用失败（{p.name}）：{e}", file=sys.stderr)
        print("    没有模型也能用：加 --no-vlm 只跑 CV；或先启动 Ollama。",
              file=sys.stderr)
        return False

    if as_json:
        print(json.dumps(res.to_dict(), ensure_ascii=False, indent=2))
    elif not quiet:
        _print_human(res)

    if save:
        paths = save_all(bgr, res)
        if not quiet:
            print(f"  已写 {paths['json']}")
            print(f"  已写 {paths['image']}")

    return res.has_target


def main(argv=None) -> int:
    fix_console()
    ap = argparse.ArgumentParser(description="红色圆形标记检测（CV + MiniCPM-V）")
    ap.add_argument("path", help="图片文件或目录")
    ap.add_argument("--no-vlm", action="store_true", help="只跑 CV，不调用模型")
    ap.add_argument("--save", action="store_true", help="输出 JSON + 标注图到 data/results")
    ap.add_argument("--json", action="store_true", help="把结果打成 JSON 到标准输出")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args(argv)

    p = Path(args.path)
    if not p.exists():
        print(f"[x] 路径不存在: {p}", file=sys.stderr)
        return 2

    use_vlm = not args.no_vlm

    # ---- 目录：批量 ----
    if p.is_dir():
        files = sorted(f for f in p.iterdir() if f.suffix.lower() in EXTS)
        if not files:
            print(f"[x] {p} 里没有图片", file=sys.stderr)
            return 2
        hits = 0
        for f in files:
            hits += _one(f, use_vlm, args.save, args.quiet, as_json=False)
            if not args.quiet:
                print("-" * 60)
        print(f"\n合计 {len(files)} 张，判为有目标 {hits} 张")
        return 0

    # ---- 单张 ----
    _one(p, use_vlm, args.save, args.quiet, args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
