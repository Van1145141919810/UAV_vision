# -*- coding: utf-8 -*-
"""bench_combined.py —— 同一批样本上，分割模型 vs VLM 正面对比

═══════════════════════════════════════════════════════════════════
要回答的问题
───────────────────────────────────────────────────────────────────
前面已经知道：
  · VLM 闭集判定 ≈ 瞎猜（准确 54.5%，误报 89%）
  · VLM 开放描述好一些（74.5%，误报 41%）
  · 预训练 SegFormer 在 CMP 上覆盖率 84.4%

但**没在同一批样本、同一个指标下正面对比过**。这里做：
  同一张 384×384 裁剪
    · 方法 A：VLM 直接问"是不是窗户"
    · 方法 B：SegFormer 算窗户像素占比，超过阈值判"是"

⚠ 必须说清的偏差
    SegFormer 是在 CMP 上训练的，所以在 CMP 样本上是**域内**，天然占优。
    因此额外在本项目自有样本（宿舍楼窗）上再跑一遍——那是它的**域外**。
    两张表一起看才不会被误导。

用法：python eval/bench_combined.py
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

EVAL = ROOT / "eval"
MODEL_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
THRESHOLDS = [0.05, 0.10, 0.20, 0.30, 0.50]


def load_segformer():
    import torch
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation
    print("加载 SegFormer …")
    proc = AutoImageProcessor.from_pretrained(MODEL_ID)
    model = AutoModelForSemanticSegmentation.from_pretrained(MODEL_ID).eval()
    ids = [i for i, n in model.config.id2label.items() if "window" in str(n).lower()]
    return proc, model, ids[0]


def win_fraction(proc, model, wid, path):
    import torch
    from PIL import Image
    img = Image.open(str(path)).convert("RGB")
    with torch.no_grad():
        logits = model(**proc(images=img, return_tensors="pt")).logits
    pred = logits.argmax(1)[0].numpy()
    # 只看中心 50%，和我们造客观真值时的口径一致
    h, w = pred.shape
    c = pred[int(h * .25):int(h * .75), int(w * .25):int(w * .75)]
    return float((c == wid).mean())


def run(labels_csv, tag, proc, model, wid):
    rows = [r for r in csv.DictReader(open(labels_csv, encoding="utf-8-sig"))
            if r.get("gt_is_window") in ("0", "1")]
    if not rows:
        print(f"{tag}: 没有有效样本")
        return None
    fracs, gts = [], []
    for r in rows:
        p = (EVAL / r["crop"]).resolve()
        if not p.exists():
            continue
        fracs.append(win_fraction(proc, model, wid, p))
        gts.append(int(r["gt_is_window"]))
    gts = np.array(gts)
    fracs = np.array(fracs)
    n = len(gts)
    print(f"\n{tag}  样本 {n}（正 {int(gts.sum())} / 负 {int((1-gts).sum())}）")
    print(f"{'阈值':<8}{'准确率':<10}{'召回':<9}{'误报':<9}{'漏报'}")
    best = None
    for th in THRESHOLDS:
        pred = (fracs >= th).astype(int)
        acc = (pred == gts).mean() * 100
        tp = int(((pred == 1) & (gts == 1)).sum())
        fp = int(((pred == 1) & (gts == 0)).sum())
        fn = int(((pred == 0) & (gts == 1)).sum())
        tn = int(((pred == 0) & (gts == 0)).sum())
        rec = 100 * tp / (tp + fn) if (tp + fn) else 0
        fpr = 100 * fp / (fp + tn) if (fp + tn) else 0
        print(f"{th:<8}{acc:<10.1f}{rec:<9.1f}{fpr:<9.1f}{100-rec:.1f}")
        if best is None or acc > best[1]:
            best = (th, acc, rec, fpr)
    print(f"  → 最佳阈值 {best[0]}：准确 {best[1]:.1f}%  召回 {best[2]:.1f}%  误报 {best[3]:.1f}%")
    return {"tag": tag, "n": n, "best_threshold": best[0],
            "acc": round(best[1], 1), "recall": round(best[2], 1),
            "fpr": round(best[3], 1)}


def main():
    proc, model, wid = load_segformer()
    out = []
    r1 = run(EVAL / "labels_clean.csv", "CMP 客观真值（SegFormer 域内）", proc, model, wid)
    if r1:
        out.append(r1)
    r2 = run(EVAL / "labels.csv", "本项目自有（SegFormer 域外）", proc, model, wid)
    if r2:
        out.append(r2)

    from datetime import datetime
    f = EVAL / "results" / f"combined_{datetime.now():%Y%m%d_%H%M%S}.json"
    f.write_text(json.dumps({"model": MODEL_ID, "results": out},
                            ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n明细 {f}")


if __name__ == "__main__":
    main()
