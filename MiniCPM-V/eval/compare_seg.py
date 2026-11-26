# -*- coding: utf-8 -*-
"""compare_seg.py —— 同一批验证图上，比较"微调前 / 微调后"

为什么必须做这一步：
    微调后报出"覆盖率 59%、纯度 70%"，看起来不高，
    但**没有基线就无从判断**——也许原始模型只有 40%。
    所以要拿同一个切分、同一个指标，把两个模型都量一遍。

指标说明（注意口径）：
    这里的"真值"是**框内区域**（框级标注，不是逐像素），
    所以纯度天然达不到 100%（框里有窗框、墙），覆盖率也天然偏低。
    这个绝对值不能被外部引用，**只能用于两者相对比较**。

用法：python eval/compare_seg.py
"""
import json
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402

LB = ROOT / "eval" / "labels_boxes"
OURS = ROOT / "eval" / "models" / "seg_ours"
BASE = "Xpitfire/segformer-finetuned-segments-cmp-facade"


def load_pairs():
    pairs = []
    for j in sorted(LB.glob("*.json")):
        try:
            d = json.loads(j.read_text(encoding="utf-8"))
        except Exception:
            continue
        p = Path(d.get("image", ""))
        boxes = d.get("boxes") or []
        if p.exists() and boxes:
            pairs.append((p, boxes))
    return pairs


def mask_from_boxes(shape, boxes):
    m = np.zeros(shape[:2], np.uint8)
    H, W = shape[:2]
    for (x, y, w, h) in boxes:
        x0, y0 = max(0, int(x)), max(0, int(y))
        x1, y1 = min(W, int(x + w)), min(H, int(y + h))
        if x1 > x0 and y1 > y0:
            m[y0:y1, x0:x1] = 1
    return m


def eval_model(model, proc, wid, pairs, tag):
    covs, precs, nboxes = [], [], []
    for p, boxes in pairs:
        img = imread_unicode(p)
        if img is None:
            continue
        pil = Image.fromarray(img[:, :, ::-1])
        with torch.no_grad():
            out = model(**proc(images=pil, return_tensors="pt"))
        logits = F.interpolate(out.logits, size=img.shape[:2], mode="nearest")
        pred = (logits.argmax(1)[0].numpy() == wid).astype(np.uint8)
        # 预测的连通域个数（用来看"过检"有没有改善）
        import cv2
        n, _, st, _ = cv2.connectedComponentsWithStats(pred, 8)
        nboxes.append(int(sum(1 for i in range(1, n) if st[i][4] >= 1500)))

        gt = mask_from_boxes(img.shape, boxes)
        inter = int((pred & gt).sum())
        covs.append(inter / max(1, int(gt.sum())))
        precs.append(inter / max(1, int(pred.sum())))
    print(f"{tag:<16}覆盖率 {100*np.mean(covs):5.1f}%   "
          f"纯度 {100*np.mean(precs):5.1f}%   "
          f"平均框数 {np.mean(nboxes):5.1f}（标注 {np.mean([len(b) for _, b in pairs]):.1f}）")
    return float(np.mean(covs)), float(np.mean(precs)), float(np.mean(nboxes))


def main():
    pairs = load_pairs()
    n_val = max(1, int(len(pairs) * 0.2))
    val = pairs[:n_val]                    # 与 finetune_seg.py 的切分保持一致
    print(f"验证集 {len(val)} 张（与微调时同一批）\n")

    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation
    proc = AutoImageProcessor.from_pretrained(BASE)

    m0 = AutoModelForSemanticSegmentation.from_pretrained(BASE).eval()
    wid = [i for i, n in m0.config.id2label.items() if "window" in str(n).lower()][0]
    r0 = eval_model(m0, proc, wid, val, "微调前(CMP)")

    if OURS.exists():
        m1 = AutoModelForSemanticSegmentation.from_pretrained(str(OURS)).eval()
        wid1 = [i for i, n in m1.config.id2label.items() if "window" in str(n).lower()][0]
        r1 = eval_model(m1, proc, wid1, val, "微调后(ours)")
        print()
        print(f"覆盖率 {100*r0[0]:.1f}% → {100*r1[0]:.1f}%  ({(r1[0]-r0[0])*100:+.1f} 点)")
        print(f"纯度   {100*r0[1]:.1f}% → {100*r1[1]:.1f}%  ({(r1[1]-r0[1])*100:+.1f} 点)")
        print(f"框数   {r0[2]:.1f} → {r1[2]:.1f}")
    else:
        print("还没有微调模型")


if __name__ == "__main__":
    main()
