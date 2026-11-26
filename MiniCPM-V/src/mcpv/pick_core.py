# -*- coding: utf-8 -*-
"""cpmv 选点核心 —— 把候选拼成编号网格，让 VLM 选"哪个是白纸上的红点"

只做"建议"，不下指令。两种用法：
  · 离线：eval/bench_vlm_vs_score.py 用它和"按分数""按白环"对比准确率
  · 在线：scripts/vlm_bridge.py 用它给实时建议

为什么是"并排网格"而不是"整图定位"：
  实测过整图 grounding，100 m 时框中心离真值 107 px；而给候选编号让它选，
  它只需要判断"哪一格是白底红圆"，这是分类不是回归，精度高得多。
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional, Sequence

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mcpv.vlm import ask_crop          # noqa: E402

# 编号用大字，VLM 对小字不敏感
_FONT = cv2.FONT_HERSHEY_SIMPLEX

PROMPT = (
    "这张图里有 {n} 个编号格，每格是一个红色候选。"
    "只有一个是【贴在白色纸上的红色圆形标记】，其余是红色墙面、衣服、文字等干扰。"
    "请只回答那个标记的编号（一个数字）。看不清就回答 0。"
)


def crop_candidates(img: np.ndarray, cands: Sequence, tile: int = 220,
                    pad_k: float = 2.2, min_half: int = 38) -> Optional[np.ndarray]:
    """把候选裁成 tile×tile 的编号网格。

    cands 里每项要有 u / v / dmaj（他的 Cand）或 cx / cy / d（我的格式）。
    返回 BGR 网格图；没有候选返回 None。
    """
    tiles = []
    H, W = img.shape[:2]
    for i, c in enumerate(cands):
        u = float(getattr(c, "u", None) if hasattr(c, "u") else c["cx"])
        v = float(getattr(c, "v", None) if hasattr(c, "v") else c["cy"])
        d = float(getattr(c, "dmaj", None) if hasattr(c, "dmaj") else c["d"])
        half = max(min_half, int(d * pad_k / 2))
        x0, y0 = int(u) - half, int(v) - half
        x1, y1 = int(u) + half, int(v) + half
        # 越界就补边（用边缘像素），不缩放，保证每格同一比例
        need = max(0, -x0, -y0, x1 - W, y1 - H)
        if need > 0:
            img = cv2.copyMakeBorder(img, need, need, need, need,
                                     cv2.BORDER_REPLICATE)
            x0 += need; y0 += need; x1 += need; y1 += need
        t = img[y0:y1, x0:x1]
        if t.size == 0:
            t = np.zeros((2 * half, 2 * half, 3), np.uint8)
        t = cv2.resize(t, (tile, tile), interpolation=cv2.INTER_LINEAR)
        # 目标框（提示 VLM 看中心）
        cv2.rectangle(t, (0, 0), (tile - 1, tile - 1), (255, 255, 255), 2)
        cv2.putText(t, str(i + 1), (8, 34), _FONT, 1.1, (0, 0, 0), 5)
        cv2.putText(t, str(i + 1), (8, 34), _FONT, 1.1, (0, 255, 255), 2)
        tiles.append(t)
    if not tiles:
        return None
    if len(tiles) == 1:
        return tiles[0]
    if len(tiles) <= 4:
        return np.hstack(tiles)
    # 多行：每行 4 格，最后一行不足就补空白格（否则 hstack 宽度不一致）
    rows = []
    for i in range(0, len(tiles), 4):
        row = tiles[i:i + 4]
        while len(row) < 4:
            row.append(np.zeros((tile, tile, 3), np.uint8))
        rows.append(np.hstack(row))
    return np.vstack(rows)


def parse_number(answer: str, n: int) -> Optional[int]:
    """从 VLM 的回答里取编号；0 或超范围 → None。"""
    import re
    nums = re.findall(r"\d+", answer or "")
    for s in nums:
        k = int(s)
        if 1 <= k <= n:
            return k
    return None


def suggest(img: np.ndarray, cands: Sequence, timeout: int = 30,
            tile: int = 220) -> dict:
    """→ {pick, n, answer, grid, elapsed}

    pick: 1-based 编号；None = VLM 没给出有效答案（此时调用方应回退到按分数）。
    """
    import time
    t0 = time.time()
    grid = crop_candidates(img, cands, tile=tile)
    out = {"pick": None, "n": len(cands), "answer": None, "grid": grid,
           "elapsed": 0.0}
    if grid is None:
        return out
    n = len(cands)
    try:
        ans, _ = ask_crop(grid, PROMPT.format(n=n), timeout=timeout)
        out["answer"] = ans
        out["pick"] = parse_number(ans, n)
    except Exception as e:
        out["answer"] = "ERR: %s" % e
    out["elapsed"] = time.time() - t0
    return out
