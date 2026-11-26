# -*- coding: utf-8 -*-
"""pred_boxes.py —— 给标注工具生成"机器预猜框"，让标注者只需删和补

思路：用现成的 CMP 预训练 SegFormer 先猜一遍，存到 eval/labels_boxes/_pred/。
标注工具里按 a 一键载入，你只做三件事：
    删掉错的（空调机位、墙面）
    补上漏的（树叶后的窗）
    拖一下边（框歪了）

⚠ 预猜只是省力，不是真值。**最终以你画的为准。**

用法：python eval/pred_boxes.py --dir D:\Desktop\window
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402

SEG_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
OUT = ROOT / "eval" / "labels_boxes" / "_pred"
MIN_AREA = 1500


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="")
    a = ap.parse_args()

    dirs = [Path(a.dir)] if a.dir else [ROOT / "data" / "samples",
                                       Path(r"D:\Desktop\window")]
    files = []
    for d in dirs:
        if d.exists():
            files += sorted(f for f in d.iterdir()
                            if f.suffix.lower() in (".jpg", ".jpeg", ".png"))
    print(f"待预猜 {len(files)} 张")

    import torch
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation

    print("加载 SegFormer …")
    proc = AutoImageProcessor.from_pretrained(SEG_ID)
    seg = AutoModelForSemanticSegmentation.from_pretrained(SEG_ID).eval()
    wid = [i for i, n in seg.config.id2label.items() if "window" in str(n).lower()][0]
    OUT.mkdir(parents=True, exist_ok=True)

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
        boxes = [[int(x), int(y), int(w), int(h)]
                 for i in range(1, n)
                 for x, y, w, h, ar in [st[i]] if ar >= MIN_AREA]
        json.dump({"image": str(p), "boxes": boxes},
                  open(OUT / f"{p.stem}.json", "w", encoding="utf-8"),
                  ensure_ascii=False)
        print(f"  {p.name:<40} {len(boxes)} 个预猜框")

    print(f"\n预猜存在 {OUT}")
    print("接着运行：python eval/label_windows.py --dir <同一个目录>")


if __name__ == "__main__":
    main()
