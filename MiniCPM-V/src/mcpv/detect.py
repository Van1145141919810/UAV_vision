# -*- coding: utf-8 -*-
"""主流程：一张图 → DetectionResult（并可选输出 JSON + 标注图）。"""

import json
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from PIL import Image, ImageOps

from . import config as C
from . import cv as cvmod
from . import vlm as vlmod
from .schema import DetectionResult


def load_bgr(path: str | Path) -> np.ndarray:
    """读图并处理 EXIF 旋转（手机拍的图带 Orientation，不处理会歪 90°）。"""
    with Image.open(str(path)) as im:
        im = ImageOps.exif_transpose(im).convert("RGB")
        arr = np.array(im)
    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)


def detect(image_path: str | Path, use_vlm: bool = True) -> DetectionResult:
    """跑完整链路。"""
    image_path = Path(image_path)
    t0 = time.time()

    bgr = load_bgr(image_path)
    h, w = bgr.shape[:2]
    cands = cvmod.find_candidates(bgr)          # 已按 C.RANK_BY 排好序
    pool = [c for c in cands if c.cdiff >= C.CDIFF_MIN]
    if C.USE_WHITE_GATE:
        pool = [c for c in pool if c.white_ok]
    shortlist = pool[:C.MAX_CANDIDATES]

    result = DetectionResult(
        image=image_path.name,
        image_path=str(image_path),
        image_size=[w, h],
        has_target=False,
        stage="no_candidate",
        n_candidates=len(cands),
        n_passed_cv=len(shortlist),
        elapsed_s=0.0,
        vlm_calls=0,
        candidates=shortlist,
    )

    if not shortlist:
        result.elapsed_s = round(time.time() - t0, 2)
        return result

    if not use_vlm:
        # 只跑 CV（做消融对比 / 模型没起时兜底）
        result.stage = "cv_only"
        result.has_target = True
        result.elapsed_s = round(time.time() - t0, 2)
        return result

    result.stage = "vlm"
    for c in shortlist[:C.VLM_MAX_CHECK]:
        patch = cvmod.crop_for_vlm(bgr, c)
        try:
            ans, _dt = vlmod.ask_crop(patch)
        except vlmod.VLMError:
            raise
        c.vlm_text = ans[:60]
        c.vlm_verdict = vlmod.parse_verdict(ans)
        result.vlm_calls += 1
        if c.vlm_verdict == "A":
            result.has_target = True
            break

    result.elapsed_s = round(time.time() - t0, 2)
    return result


def annotate(bgr: np.ndarray, res: DetectionResult) -> np.ndarray:
    """在图上画框 + 画结论。"""
    img = bgr.copy()
    h, w = img.shape[:2]
    thick = max(2, int(min(w, h) / 400))

    for c in res.candidates:
        if c.vlm_verdict == "A":
            color, tag = (0, 200, 0), "TARGET A"
        elif c.vlm_verdict == "B":
            color, tag = (0, 165, 255), "B"
        else:
            color, tag = (128, 128, 128), "cv-cand"

        x0, y0, x1, y1 = c.bbox_px
        cv2.rectangle(img, (x0, y0), (x1, y1), color, thick)
        cv2.putText(img, f"{tag} c{c.circularity:.2f}", (x0, max(16, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, max(0.5, min(w, h) / 1200), color, thick)

    verdict = "TARGET FOUND" if res.has_target else "NO TARGET"
    bg = (0, 160, 0) if res.has_target else (60, 60, 60)
    cv2.rectangle(img, (0, 0), (w, int(34 * max(1.0, w / 1200))), bg, -1)
    txt = (f"{verdict}  |  {res.image}  |  cand={res.n_candidates} "
           f"cv={res.n_passed_cv}  {res.elapsed_s:.1f}s")
    cv2.putText(img, txt, (8, int(24 * max(1.0, w / 1200))),
                cv2.FONT_HERSHEY_SIMPLEX, max(0.55, min(w, h) / 1100), (255, 255, 255), 2)
    return img


def save_all(bgr: np.ndarray, res: DetectionResult,
             out_dir: Optional[Path] = None,
             stem: Optional[str] = None) -> dict:
    """把 JSON 和标注图写到 data/results，返回两个路径。"""
    out_dir = Path(out_dir or C.RESULTS_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = stem or Path(res.image).stem

    jpath = out_dir / f"{stem}.json"
    ipath = out_dir / f"{stem}_annotated.jpg"

    jpath.write_text(json.dumps(res.to_dict(), ensure_ascii=False, indent=2),
                     encoding="utf-8")
    cvmod.imwrite_unicode(ipath, annotate(bgr, res))
    return {"json": str(jpath), "image": str(ipath)}
