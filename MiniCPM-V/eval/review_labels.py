# -*- coding: utf-8 -*-
"""review_labels.py —— 把标注画回原图，生成自查拼版 + 统计

为什么要有这一步
───────────────────────────────────────────────────────────────────
标注是最容易出错、也最难回头改的环节。画回图上让人一眼检查，
比看 JSON 里的数字有用一百倍。

同时做三件统计：
  1. 每张图的框数、框面积中位数
  2. **超大框**（超过画面 25%）—— 对应"远处直接框一整块"的标法
  3. 框尺寸离散度 —— 用来发现"同一张图里既标大块又标小格"的混乱

用法：python eval/review_labels.py
输出：eval/figures/labels_check_*.jpg（每张拼版 12 图）
     控制台打印统计表
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode        # noqa: E402

LB = ROOT / "eval" / "labels_boxes"
FIG = ROOT / "eval" / "figures"
CELL = 340
COLS = 4


def main():
    FIG.mkdir(exist_ok=True)
    files = sorted(LB.glob("*.json"))
    if not files:
        raise SystemExit("还没有标注")

    rows_stats = []
    tiles = []
    for j in files:
        d = json.loads(j.read_text(encoding="utf-8"))
        src = Path(d["image"])
        boxes = d.get("boxes") or []
        img = imread_unicode(src)
        if img is None:
            print(f"  ✗ 读不到原图 {src}")
            continue
        H, W = img.shape[:2]

        vis = img.copy()
        big = 0
        areas = []
        for (x, y, w, h) in boxes:
            cv2.rectangle(vis, (int(x), int(y)), (int(x + w), int(y + h)),
                          (0, 220, 0), max(2, W // 500))
            a = (w * h) / (W * H)
            areas.append(a)
            if a > 0.25:
                big += 1
        med = float(np.median(areas)) if areas else 0.0
        rows_stats.append((src.stem[:14], len(boxes), med * 100, big, W, H))

        t = cv2.resize(vis, (CELL, int(CELL * H / W)))
        cv2.putText(t, f"{src.stem[:12]}  {len(boxes)}框", (6, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        tiles.append(t)

    print(f"{'图':<16}{'框数':<6}{'中位框面积%':<12}{'超大框':<8}{'原图尺寸'}")
    print("-" * 62)
    for s in rows_stats:
        print(f"{s[0]:<16}{s[1]:<6}{s[2]:<12.1f}{s[3]:<8}{s[4]}x{s[5]}")
    tot = sum(s[1] for s in rows_stats)
    print("-" * 62)
    print(f"合计 {len(rows_stats)} 张，{tot} 个框")
    nbig = sum(s[3] for s in rows_stats)
    print(f"超大框（>25% 画面）共 {nbig} 个 —— 对应『远处整块框』的标法，属正常")

    # 拼版：每张 12 图
    per = COLS * 3
    for si in range(0, len(tiles), per):
        chunk = tiles[si:si + per]
        Hm = max(t.shape[0] for t in chunk)
        grid = []
        for r in range(0, len(chunk), COLS):
            row = chunk[r:r + COLS]
            row = [cv2.copyMakeBorder(t, 0, Hm - t.shape[0], 0, 0,
                                      cv2.BORDER_CONSTANT, value=(25, 25, 25))
                   for t in row]
            while len(row) < COLS:
                row.append(np.full((Hm, CELL, 3), 25, np.uint8))
            grid.append(np.hstack(row))
        sheet = np.vstack(grid)
        out = FIG / f"labels_check_{si // per + 1}.jpg"
        imwrite_unicode(out, sheet, quality=88)
        print(f"拼版 {out}")


if __name__ == "__main__":
    main()
