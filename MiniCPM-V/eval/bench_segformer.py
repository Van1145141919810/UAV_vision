# -*- coding: utf-8 -*-
"""bench_segformer.py —— 用训练过的语义分割替代前端，能拿回多少？

═══════════════════════════════════════════════════════════════════
要验证的命题
───────────────────────────────────────────────────────────────────
文档评审时的关键判断是："前端窗框检测是当前瓶颈（覆盖率 2.2% / 纯度 4.4%），
换训练过的分割能到 90%+。"

这个脚本就是去验证这句话，而不是假设它成立。

做法：
  1. 用 HF 上**已经在 CMP 立面库上微调过**的 SegFormer
     （Xpitfire/segformer-finetuned-segments-cmp-facade）
  2. 在 CMP 图上跑推理，取 "window" 类
  3. 与 CMP 自带的逐像素标注比，算三个指标：
       IoU           —— 分割质量的标准指标
       覆盖率 recall —— 真窗像素里有多少被找到（与我们 CV 的 2.2% 同口径）
       框内纯度 prec —— 预测为窗的像素里有多少真是窗（同 CV 的 4.4%）

⚠ 这是**域内验证**（模型就是在 CMP 上训练的），所以数字会偏乐观。
   它能回答"这类方法的天花板"，但不能回答"换到我们宿舍楼能不能行"。
   后者需要重新训练+真实数据，另立项。

用法：python eval/bench_segformer.py --n 30
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

CMP = ROOT / "data" / "external" / "cmp" / "base"
WIN_CLS = 3                     # CMP 调色板：3 = window
MODEL_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--model", default=MODEL_ID)
    args = ap.parse_args()

    import torch
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"设备 {dev} | 模型 {args.model}")
    proc = AutoImageProcessor.from_pretrained(args.model)
    model = AutoModelForSemanticSegmentation.from_pretrained(args.model).to(dev).eval()
    id2label = model.config.id2label
    print("标签表:", id2label)

    # 找到"窗户"这一类的编号
    win_ids = [i for i, n in id2label.items() if "window" in str(n).lower()]
    if not win_ids:
        raise SystemExit("标签表里没有 window，检查模型")
    win_id = win_ids[0]
    print(f"→ 用类别 {win_id} = {id2label[win_id]} 作为窗户")

    imgs = sorted(CMP.glob("*.jpg"))[:args.n]
    ious, covs, precs = [], [], []

    for p in imgs:
        mp = p.with_suffix(".png")
        if not mp.exists():
            continue
        img = Image.open(p).convert("RGB")
        gt = np.array(Image.open(mp))
        gt_win = (gt == WIN_CLS)
        if gt_win.sum() < 500:
            continue

        inp = proc(images=img, return_tensors="pt").to(dev)
        with torch.no_grad():
            out = model(**inp)
        logits = torch.nn.functional.interpolate(
            out.logits, size=gt.shape, mode="bilinear", align_corners=False)
        pred = logits.argmax(1)[0].cpu().numpy()
        pred_win = (pred == win_id)

        inter = int((pred_win & gt_win).sum())
        union = int((pred_win | gt_win).sum())
        iou = inter / union if union else 0.0
        cov = inter / int(gt_win.sum()) if gt_win.sum() else 0.0
        prec = inter / int(pred_win.sum()) if pred_win.sum() else 0.0

        ious.append(iou)
        covs.append(cov)
        precs.append(prec)
        print(f"  {p.stem}  IoU {iou*100:5.1f}%  覆盖 {cov*100:5.1f}%  纯度 {prec*100:5.1f}%")

    if not ious:
        raise SystemExit("没有有效样本")
    n = len(ious)
    print(f"\n共 {n} 张 CMP 图")
    print(f"  IoU 均值        : {100*np.mean(ious):.1f}%")
    print(f"  窗像素覆盖率    : {100*np.mean(covs):.1f}%   （现有 CV: 2.2%）")
    print(f"  预测框内纯度    : {100*np.mean(precs):.1f}%   （现有 CV: 4.4%）")

    import json
    from datetime import datetime
    out = ROOT / "eval" / "results" / f"segformer_{datetime.now():%Y%m%d_%H%M%S}.json"
    out.write_text(json.dumps({
        "model": args.model, "n": n,
        "iou": round(100*float(np.mean(ious)), 1),
        "coverage": round(100*float(np.mean(covs)), 1),
        "purity": round(100*float(np.mean(precs)), 1),
        "note": "域内验证（模型在 CMP 上训练过），不代表跨域能力",
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n明细 {out}")


if __name__ == "__main__":
    main()
