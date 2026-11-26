# -*- coding: utf-8 -*-
"""bench_filter.py —— 实测"分割出框 + VLM 复核"这条流水线

═══════════════════════════════════════════════════════════════════
要验证的假设（来自用户）
───────────────────────────────────────────────────────────────────
"分割会框出一些不是窗户的东西；再让 CPMV 过一遍描述，
  就能筛掉那些假的，留下真窗户。"

这需要回答三件事：
  1. **分割到底会不会把不是窗户的判成窗户？**（会的话占多少）
  2. **VLM 复核能筛掉多少假框？**（precision 提升多少）
  3. **代价是什么？**（会不会把真窗户也筛掉 → recall 掉多少）

═══════════════════════════════════════════════════════════════════
方法
───────────────────────────────────────────────────────────────────
数据：CMP 立面图 20 张（有逐像素标注，可真值）
流程：
  SegFormer → 窗户掩膜 → 连通域 → 取面积最大的前 K 个框
  每个框：中心 50% 区域的真实窗占比（来自人工标注）
      ≥0.60  → 真窗户（正例）
      ≤0.05  → 假框（负例，分割错了）
      中间    → 歧义，不计入指标
  再对每个框裁 384×384 送 VLM，用关键词判"它说是窗户吗"

对比：
  A. 分割出框（全收）
  B. 分割出框 + VLM 复核（VLM 说不是的删掉）

═══════════════════════════════════════════════════════════════════
⚠ 偏差
  SegFormer 在 CMP 上训练过 → 域内，假框率会偏低（乐观）。
  换到真实场景假框会更多，那么"VLM 复核"的价值也会更大。

用法：python eval/bench_filter.py --images 20 --max-boxes 6
"""
import argparse
import base64
import json
import sys
import urllib.request
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imwrite_unicode                        # noqa: E402

CMP = ROOT / "data" / "external" / "cmp" / "base"
EVAL = ROOT / "eval"
SEG_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
VLM_MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"
P_DESC = "用不超过十个字描述画面正中心的主要物体。"
WIN_CLS = 3


def vlm_desc(crop_bgr):
    ok, buf = cv2.imencode(".jpg", crop_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    b64 = base64.b64encode(buf.tobytes()).decode()
    body = json.dumps({"model": VLM_MODEL, "prompt": P_DESC, "images": [b64],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=180) as r:
        return json.loads(r.read()).get("response", "").strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", type=int, default=20)
    ap.add_argument("--max-boxes", type=int, default=6)
    ap.add_argument("--min-area", type=int, default=1200)
    a = ap.parse_args()

    import torch
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation

    print("加载 SegFormer …")
    proc = AutoImageProcessor.from_pretrained(SEG_ID)
    seg = AutoModelForSemanticSegmentation.from_pretrained(SEG_ID).eval()
    wid = [i for i, n in seg.config.id2label.items() if "window" in str(n).lower()][0]

    imgs = sorted(CMP.glob("*.jpg"))[:a.images]
    outdir = EVAL / "filter_crops"
    outdir.mkdir(exist_ok=True)

    records = []
    for p in imgs:
        mp = p.with_suffix(".png")
        if not mp.exists():
            continue
        img = Image.open(p).convert("RGB")
        gt = np.array(Image.open(mp))
        rgb = np.array(img)[:, :, ::-1]

        with torch.no_grad():
            logits = seg(**proc(images=img, return_tensors="pt")).logits
        logits = torch.nn.functional.interpolate(
            logits, size=gt.shape, mode="bilinear", align_corners=False)
        mask = (logits.argmax(1)[0].numpy() == wid).astype(np.uint8)

        # 连通域 → 框
        n, lab, st, cen = cv2.connectedComponentsWithStats(mask, 8)
        boxes = []
        for i in range(1, n):
            x, y, w, h, area = st[i]
            if area < a.min_area:
                continue
            boxes.append((area, x, y, w, h))
        boxes.sort(reverse=True)
        boxes = boxes[:a.max_boxes]

        for k, (area, x, y, w, h) in enumerate(boxes):
            side = max(w, h)
            half = max(48, int(side * 0.75))
            cx, cy = x + w // 2, y + h // 2
            H, W = gt.shape[:2]
            x0 = max(0, min(W - 2 * half, cx - half))
            y0 = max(0, min(H - 2 * half, cy - half))
            sub_gt = gt[y0:y0 + 2 * half, x0:x0 + 2 * half]
            patch = rgb[y0:y0 + 2 * half, x0:x0 + 2 * half]
            if patch.size == 0 or sub_gt.size == 0:
                continue
            hh, ww = sub_gt.shape
            c = sub_gt[int(hh * .25):int(hh * .75), int(ww * .25):int(ww * .75)]
            frac = float((c == WIN_CLS).mean())

            crop = cv2.resize(patch, (384, 384), interpolation=cv2.INTER_AREA)
            name = f"{p.stem}_b{k}.jpg"
            imwrite_unicode(outdir / name, crop, quality=92)
            desc = vlm_desc(crop)

            records.append({
                "image": p.stem, "box": [int(x), int(y), int(w), int(h)],
                "win_frac": round(frac, 3),
                "gt": ("1" if frac >= 0.60 else ("0" if frac <= 0.05 else "?")),
                "desc": desc[:20],
                "vlm_says_win": int("窗" in desc),
                "crop": name,
            })
            print(f"  {p.stem} b{k}  真窗占比 {frac:.2f}  模型说: {desc[:16]}", flush=True)

    # ── 统计 ────────────────────────────────────────────────
    valid = [r for r in records if r["gt"] in ("0", "1")]
    n = len(valid)
    if not n:
        raise SystemExit("没有有效样本（全部落在歧义区间）")

    def stats(rows, keep):
        tp = sum(1 for r in rows if r["gt"] == "1" and keep(r))
        fp = sum(1 for r in rows if r["gt"] == "0" and keep(r))
        fn = sum(1 for r in rows if r["gt"] == "1" and not keep(r))
        prec = 100 * tp / (tp + fp) if (tp + fp) else 0
        rec = 100 * tp / (tp + fn) if (tp + fn) else 0
        return tp, fp, fn, prec, rec

    print("\n" + "=" * 62)
    print(f"分割共出框 {len(records)} 个（歧义 {len(records)-n} 个不计）")
    print(f"其中真窗户 {sum(1 for r in valid if r['gt']=='1')} 个，"
          f"**假框 {sum(1 for r in valid if r['gt']=='0')} 个**")

    tp, fp, fn, prec, rec = stats(valid, lambda r: True)
    print(f"\nA. 分割出框全收：      精确率 {prec:.1f}%  召回 {rec:.1f}%  (TP={tp} FP={fp} FN={fn})")

    tp2, fp2, fn2, prec2, rec2 = stats(valid, lambda r: r["vlm_says_win"] == 1)
    print(f"B. 再用 VLM 复核筛一遍：精确率 {prec2:.1f}%  召回 {rec2:.1f}%  (TP={tp2} FP={fp2} FN={fn2})")
    print(f"\n   假框被清掉 {fp - fp2} 个，误删真窗户 {tp - tp2} 个")

    f = EVAL / "results" / f"filter_{datetime.now():%Y%m%d_%H%M%S}.json"
    f.write_text(json.dumps({
        "seg": SEG_ID, "vlm": VLM_MODEL,
        "boxes": len(records), "valid": n,
        "false_boxes": sum(1 for r in valid if r["gt"] == "0"),
        "A_seg_only": {"precision": round(prec, 1), "recall": round(rec, 1),
                       "tp": tp, "fp": fp, "fn": fn},
        "B_seg_plus_vlm": {"precision": round(prec2, 1), "recall": round(rec2, 1),
                           "tp": tp2, "fp": fp2, "fn": fn2},
        "records": records,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n明细 {f}")


if __name__ == "__main__":
    main()
