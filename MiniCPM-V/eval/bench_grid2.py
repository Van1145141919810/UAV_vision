# -*- coding: utf-8 -*-
"""bench_grid2.py —— 几何校正 v2：列投票 + 明暗，替代 v1 的"行内高度一致"

═══════════════════════════════════════════════════════════════════
v1 为什么失败（保留这个教训）
───────────────────────────────────────────────────────────────────
v1 按"同一行里高度接近"来筛。结果：
    宿舍楼的**空调机位**比窗户更规整（同尺寸、同高度），
    于是 v1 把空调留下、把真窗户删掉。
    指标"行内高度离散度"还显示大幅改善（0.44→0.25）——
    **指标在奖励错误的行为**。这是典型的目标函数选错。

═══════════════════════════════════════════════════════════════════
v2 换两个更贴近物理的线索
───────────────────────────────────────────────────────────────────
1. **垂直列重复**：同一栋楼的窗在竖直方向上一层层重复出现。
   一个框若在"同一 x 位置、不同楼层"还有别的框，它更像窗。
2. **亮度**：玻璃是暗的（透射/反射），空调机位是亮白色金属。
   框内平均亮度越接近墙面（很亮）越可疑。

打分：score = 列支持数（-0~3） + 亮度分（-1~1）
只看排序，不硬删——输出排序后的前 K 个，让人肉眼看是否合理。

用法：python eval/bench_grid2.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode        # noqa: E402

SEG_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
FILES = sorted((ROOT / "data" / "samples").glob("win0*.jpg")) + \
        sorted((ROOT / "data" / "samples").glob("win1*.jpg"))
MIN_AREA = 1200
CELL = 380


def column_support(boxes, tol_ratio=0.35):
    """每个框在"同一列、不同行"上还有几个伙伴。"""
    if not boxes:
        return []
    med_w = float(np.median([b[2] for b in boxes]))
    med_h = float(np.median([b[3] for b in boxes]))
    out = []
    for i, b in enumerate(boxes):
        cx, cy = b[0] + b[2] / 2, b[1] + b[3] / 2
        n = 0
        for j, o in enumerate(boxes):
            if i == j:
                continue
            ox, oy = o[0] + o[2] / 2, o[1] + o[3] / 2
            if abs(cx - ox) < tol_ratio * med_w and abs(cy - oy) > tol_ratio * med_h:
                n += 1
        out.append(n)
    return out


def brightness(img, b):
    x, y, w, h = b
    H, W = img.shape[:2]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(W, x + w), min(H, y + h)
    if x1 <= x0 or y1 <= y0:
        return 255.0
    patch = img[y0:y1, x0:x1]
    return float(cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY).mean())


def main():
    import torch
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation

    print("加载 SegFormer …")
    proc = AutoImageProcessor.from_pretrained(SEG_ID)
    seg = AutoModelForSemanticSegmentation.from_pretrained(SEG_ID).eval()
    wid = [i for i, n in seg.config.id2label.items() if "window" in str(n).lower()][0]

    tiles = []
    print(f"{'图':<10}{'原框':<6}{'取前K':<6}{'亮度均值(窗/AC)'}")
    print("-" * 56)
    for p in FILES[:8]:
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
        boxes = [(int(x), int(y), int(w), int(h))
                 for i in range(1, n)
                 for x, y, w, h, a in [st[i]] if a >= MIN_AREA]
        if not boxes:
            continue

        sup = column_support(boxes)
        bright = [brightness(img, b) for b in boxes]
        b_arr = np.array(bright)
        # 亮度分：比本图亮度中位数暗的加分
        med_b = float(np.median(b_arr))
        score = [s * 1.0 + (1.0 if br < med_b else -1.0)
                 for s, br in zip(sup, bright)]
        order = np.argsort(-np.array(score))
        k = max(6, int(len(boxes) * 0.55))
        keep_idx = set(order[:k].tolist())

        vis = img.copy()
        for i, (x, y, w, h) in enumerate(boxes):
            col = (80, 220, 80) if i in keep_idx else (60, 60, 235)
            cv2.rectangle(vis, (x, y), (x + w, y + h), col, 3)
        kb = [bright[i] for i in keep_idx]
        db = [bright[i] for i in range(len(boxes)) if i not in keep_idx]
        print(f"{p.stem:<10}{len(boxes):<6}{k:<6}"
              f"{np.mean(kb):.0f} / {np.mean(db):.0f}" if db else
              f"{p.stem:<10}{len(boxes):<6}{k:<6}{np.mean(kb):.0f} / -")

        t = cv2.resize(vis, (CELL, int(CELL * img.shape[0] / img.shape[1])))
        cv2.putText(t, f"{p.stem} keep {k}/{len(boxes)}", (6, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        tiles.append(t)

    tiles = [cv2.resize(t, (int(t.shape[1] * CELL / t.shape[0]), CELL)) for t in tiles]
    rows = []
    for i in range(0, len(tiles), 4):
        row = tiles[i:i + 4]
        while len(row) < 4:
            row.append(np.zeros_like(row[0]))
        rows.append(np.hstack(row))
    out = ROOT / "eval" / "figures" / "grid_correct2.jpg"
    imwrite_unicode(out, np.vstack(rows), quality=90)
    print(f"\n绿=保留  红=丢弃\n拼版 {out}")


if __name__ == "__main__":
    main()
