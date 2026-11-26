# -*- coding: utf-8 -*-
"""contact_sheet.py —— 把裁剪图拼成大图，方便一眼看完并标注。

38 张裁剪拼成 2~3 张拼版，每格下面写 image_id，用来做人眼标注（GT）。
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode                        # noqa: E402

CROPS = ROOT / "eval" / "crops"
OUT = ROOT / "eval" / "sheets"

COLS, CELL = 6, 220
PAD_LABEL = 22


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    files = sorted(CROPS.glob("*.jpg"))
    per = COLS * 5
    for si in range(0, len(files), per):
        chunk = files[si:si + per]
        rows = math.ceil(len(chunk) / COLS)
        sheet = np.full((rows * (CELL + PAD_LABEL), COLS * CELL, 3), 32, np.uint8)
        for i, f in enumerate(chunk):
            r, c = divmod(i, COLS)
            img = imread_unicode(f)   # cv2.imread 读不了中文路径
            if img is None:
                continue
            img = cv2.resize(img, (CELL - 4, CELL - 4), interpolation=cv2.INTER_AREA)
            y = r * (CELL + PAD_LABEL)
            x = c * CELL
            sheet[y + 2:y + 2 + img.shape[0], x + 2:x + 2 + img.shape[1]] = img
            name = f.stem
            cv2.putText(sheet, name, (x + 4, y + CELL + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        out = OUT / f"sheet{si // per + 1}.jpg"
        imwrite_unicode(out, sheet, quality=92)
        print(f"{out}  ({len(chunk)} 张)")


if __name__ == "__main__":
    main()
