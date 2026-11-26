# -*- coding: utf-8 -*-
"""show_summary.py —— 打印最近一次评测的关键指标（给 完整评测.cmd 收尾用）"""
import json
import sys
from pathlib import Path

R = Path(__file__).resolve().parent / "results"


def main():
    fs = [f for f in sorted(R.glob("*.json")) if not f.name.startswith(("pixels_", "prompt_",
                                                                        "load_", "segformer_",
                                                                        "filter_", "combined_"))]
    if not fs:
        print("还没有评测结果")
        return 1
    d = json.loads(fs[-1].read_text(encoding="utf-8"))
    w = d.get("is_window", {})
    lat = d.get("latency_s", {})
    mode = "开放描述" if d.get("mode") == "open" else "闭集 A/B"
    print()
    print("=" * 58)
    print(f"  最近一次评测：{d.get('stamp', '')}   模式 {mode}")
    print("=" * 58)
    print(f"  样本（有效窗户判定）  {w.get('n', 0)}")
    print(f"  准确率                {w.get('acc', 'n/a')}")
    print(f"  召回（真窗被认出）    {w.get('recall', 'n/a')}")
    print(f"  **误报率**            {w.get('false_positive_rate', 'n/a')}")
    print(f"  漏报率                {w.get('false_negative_rate', 'n/a')}")
    print(f"  精确率                {w.get('precision', 'n/a')}")
    print(f"  混淆矩阵  TP={w.get('tp', 0)}  FP={w.get('fp', 0)}  "
          f"FN={w.get('fn', 0)}  TN={w.get('tn', 0)}")
    if lat:
        print(f"  时延 p50 / p95        {lat.get('p50', '?')} s / {lat.get('p95', '?')} s")
    print("=" * 58)
    print(f"  明细 {fs[-1].name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
