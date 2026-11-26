# -*- coding: utf-8 -*-
"""bench_seg_own.py —— 把我们自己拍的照片跑一遍分割，看框画得对不对

═══════════════════════════════════════════════════════════════════
为什么必须单独测
───────────────────────────────────────────────────────────────────
之前分割只在 CMP（欧洲街景立面，它的训练集）上测过：
  · 像素覆盖 84.4%、纯度 89.1%、IoU 76.8%
  · 框级 59/59 全是真窗（0 假框）

但那是**域内**。我们真正的场景是：中国宿舍楼、树遮挡、逆光、远距离。
模型在那里会不会画出一堆假框？**完全没测过。**

本脚本不训练、不调参，只是把框画出来给人看——
因为"框画得对不对"最终必须靠眼睛判断。

输出：eval/figures/seg_own.jpg（每张图叠加检测框 + 窗占比）
用法：python eval/bench_seg_own.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode        # noqa: E402

SEG_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
SRC_DIRS = [ROOT / "data" / "samples", Path(r"D:\Desktop\window")]
MIN_AREA = 800
CELL = 300


def main():
    import torch
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation

    print("加载 SegFormer …")
    proc = AutoImageProcessor.from_pretrained(SEG_ID)
    seg = AutoModelForSemanticSegmentation.from_pretrained(SEG_ID).eval()
    wid = [i for i, n in seg.config.id2label.items() if "window" in str(n).lower()][0]

    files = []
    for d in SRC_DIRS:
        if d.exists():
            files += sorted(f for f in d.iterdir()
                            if f.suffix.lower() in (".jpg", ".jpeg", ".png"))
    files = files[:12]
    print(f"待测 {len(files)} 张")

    tiles, stats = [], []
    for p in files:
        img = imread_unicode(p)
        if img is None:
            continue
        pil = Image.fromarray(img[:, :, ::-1])
        with torch.no_grad():
            logits = seg(**proc(images=pil, return_tensors="pt")).logits
        logits = torch.nn.functional.interpolate(
            logits, size=(img.shape[0], img.shape[1]), mode="bilinear",
            align_corners=False)
        mask = (logits.argmax(1)[0].numpy() == wid).astype(np.uint8)

        n, lab, st, cen = cv2.connectedComponentsWithStats(mask, 8)
        vis = img.copy()
        boxes = 0
        for i in range(1, n):
            x, y, w, h, area = st[i]
            if area < MIN_AREA:
                continue
            boxes += 1
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 170, 255), 3)
        # 覆盖率粗估
        cov = float(mask.mean()) * 100
        stats.append((p.name, boxes, cov))
        t = cv2.resize(vis, (CELL, CELL))   # 统一尺寸，方便拼版
        cv2.putText(t, f"{boxes} boxes  {cov:.1f}%px", (6, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        tiles.append(t)
        print(f"  {p.name:<40} 框 {boxes:>2}  窗像素占比 {cov:5.1f}%")

    # 拼版（统一高度）
    rows = []
    per = 4
    for i in range(0, len(tiles), per):
        row = tiles[i:i + per]
        while len(row) < per:
            row.append(np.zeros_like(row[0]))
        rows.append(np.hstack(row))
    sheet = np.vstack(rows)
    out = ROOT / "eval" / "figures" / "seg_own.jpg"
    out.parent.mkdir(exist_ok=True)
    imwrite_unicode(out, sheet, quality=90)
    print(f"\n拼版 {out}")


if __name__ == "__main__":
    main()
