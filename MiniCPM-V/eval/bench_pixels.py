# -*- coding: utf-8 -*-
"""bench_pixels.py —— "窗宽剩几个像素"时，模型还有没有**区分能力**？（文档 R01）

═══════════════════════════════════════════════════════════════════
★ 为什么必须正负一起测（第一版跑废的教训）
───────────────────────────────────────────────────────────────────
只测"真窗户图缩到 N 像素还能不能答 A"是没意义的：
这个模型是"好好先生"，**看不出也默认答 A**，于是曲线永远是 100%。

要测的是**区分能力**：同样缩到 N 像素，
  正例（真窗户）它答 A 吗？
  负例（墙/树/车）它**还答 B 吗**？
两者一起掉，才说明"信息不够了"。

═══════════════════════════════════════════════════════════════════
换算
───────────────────────────────────────────────────────────────────
  N = f × 窗宽 / Z   ⇒   Z = f × 2.0 / N     （f=1566 px @1920 宽，窗宽按 2 m）
  N=384→8 m   N=96→33 m   N=48→65 m   N=24→131 m   N=12→261 m

⚠ 合成降采样，非真实远距成像；只回答"信息量够不够"。

用法：python eval/bench_pixels.py [--pos 13] [--neg 20]
"""
import argparse
import base64
import csv
import json
import sys
import urllib.request
from datetime import datetime
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode      # noqa: E402

EVAL = ROOT / "eval"
MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"
P_GATE = ("只看画面正中心的主要物体。它是窗户吗？"
          "只回答一个字母：A=是窗户，B=不是窗户。")

SIDES = [384, 96, 48, 24, 12]
F_PX, WIN_W_M = 1566.0, 2.0


def ask(p: Path, prompt: str):
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(p.read_bytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read()).get("response", "").strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pos", type=int, default=13)
    ap.add_argument("--neg", type=int, default=20)
    args = ap.parse_args()

    with open(EVAL / "labels.csv", encoding="utf-8-sig") as f:
        rows = [r for r in csv.DictReader(f) if r["kind"] != "full_image"]
    pos = [r for r in rows if r["gt_is_window"] == "1"][:args.pos]
    neg = [r for r in rows if r["gt_is_window"] == "0"][:args.neg]
    items = [(r, 1) for r in pos] + [(r, 0) for r in neg]
    print(f"正例 {len(pos)} + 负例 {len(neg)} = {len(items)} 张 × {len(SIDES)} 档 "
          f"= {len(items)*len(SIDES)} 次调用")

    tmp = EVAL / "pixel_tmp" / "_t.jpg"
    tmp.parent.mkdir(exist_ok=True)
    tab = {n: {"tp": 0, "fn": 0, "tn": 0, "fp": 0} for n in SIDES}
    detail = []

    for r, is_win in items:
        img = imread_unicode((EVAL / r["crop"]).resolve())
        for n in SIDES:
            small = cv2.resize(img, (n, n), interpolation=cv2.INTER_AREA)
            big = cv2.resize(small, (384, 384), interpolation=cv2.INTER_NEAREST)
            imwrite_unicode(tmp, big, quality=95)
            ans = ask(tmp, P_GATE)
            said_A = next((c for c in ans.upper() if c in "AB"), "?") == "A"
            k = tab[n]
            if is_win and said_A:
                k["tp"] += 1
            elif is_win and not said_A:
                k["fn"] += 1
            elif not is_win and said_A:
                k["fp"] += 1
            else:
                k["tn"] += 1
            detail.append({"image": r["image_id"], "is_window": is_win,
                           "side": n, "said_A": int(said_A)})
        print(f"  {r['image_id']:<12} 完成", flush=True)

    curve = []
    for n in SIDES:
        k = tab[n]
        npos, nneg = k["tp"] + k["fn"], k["tn"] + k["fp"]
        recall = 100.0 * k["tp"] / npos if npos else 0
        spec = 100.0 * k["tn"] / nneg if nneg else 0
        acc = 100.0 * (k["tp"] + k["tn"]) / (npos + nneg) if (npos + nneg) else 0
        curve.append({"side_px": n, "dist_m": round(F_PX * WIN_W_M / n, 1),
                      "recall_正例答A": round(recall, 1),
                      "specificity_负例答B": round(spec, 1),
                      "acc": round(acc, 1),
                      "tp": k["tp"], "fn": k["fn"], "tn": k["tn"], "fp": k["fp"]})
        print(f"  {n:>4}px ≈{F_PX*WIN_W_M/n:>6.1f} m  召回{recall:5.1f}%  "
              f"特异{spec:5.1f}%  准确{acc:5.1f}%", flush=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = EVAL / "results" / f"pixels_{stamp}.json"
    out.write_text(json.dumps({"model": MODEL,
                               "note": "合成降采样；正负配对测区分能力",
                               "window_w_m": WIN_W_M, "f_px": F_PX,
                               "curve": curve, "detail": detail},
                              ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.unlink(missing_ok=True)
    print(f"\n明细 {out}")


if __name__ == "__main__":
    main()
