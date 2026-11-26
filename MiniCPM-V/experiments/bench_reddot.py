# -*- coding: utf-8 -*-
"""
bench_reddot.py —— 批量问一组图片"有没有红色圆点"，把结果记成表。

用法：
    python bench_reddot.py 图片目录
    python bench_reddot.py 图片目录 --question "自定义问题"
    python bench_reddot.py 图片目录 --out 结果.csv

输出：
    · 屏幕上的表（每张图一行）
    · 一个 csv（utf-8-sig，Excel 直接打开不乱码）

注意：
    1. 每张图第一次要付 ~6 秒（看图），别以为卡住。
    2. 同一张图重复问只算一次（Ollama 有缓存），所以这个脚本每张只问一次。
       要测"稳定性"得换问题措辞，见 --repeat-variant。
"""

import sys
import os
import csv
import json
import time
import base64
import urllib.request
import urllib.error

HOST = "http://127.0.0.1:11434"
MODEL = "openbmb/minicpm-v4.7:1b"

DEFAULT_Q = "这张图里有没有红色圆点？如果有，指出它在画面的什么位置；如果没有，请直接回答“没有”。"

EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def fix_console():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
        except Exception:
            pass
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass


def ask(img, prompt):
    raw = open(img, "rb").read()
    payload = {
        "model": MODEL,
        "prompt": prompt,
        "images": [base64.b64encode(raw).decode("ascii")],
        "stream": False,
        "options": {"temperature": 0},
        "keep_alive": "30m",
        "think": False,
    }
    req = urllib.request.Request(
        HOST + "/api/generate",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    t0 = time.time()
    r = json.loads(urllib.request.urlopen(req, timeout=900).read())
    return r, time.time() - t0


def verdict(ans):
    """粗判：它到底说了'有'还是'没有'。原文照存，人工复核用。"""
    a = (ans or "").strip()
    head = a[:24]
    if head.startswith("没有") or head.startswith("无"):
        return "没有"
    if "没有" in head and "但" in head:
        return "矛盾"
    if "有" in head:
        return "有"
    return "?"


def main():
    fix_console()
    argv = sys.argv[1:]
    if not argv:
        print(__doc__)
        return 1

    folder = argv[0]
    q = DEFAULT_Q
    if "--question" in argv:
        q = argv[argv.index("--question") + 1]
    out_csv = None
    if "--out" in argv:
        out_csv = argv[argv.index("--out") + 1]

    if not os.path.isdir(folder):
        print(f"[x] 不是目录: {folder}")
        return 1

    files = sorted(
        f for f in os.listdir(folder)
        if f.lower().endswith(EXTS) and not f.startswith("_")
    )
    if not files:
        print("[x] 目录里没有图片")
        return 1

    print(f"共 {len(files)} 张，问题：{q}\n")
    rows = []
    for f in files:
        p = os.path.join(folder, f)
        try:
            r, dt = ask(p, q)
        except urllib.error.URLError as e:
            print(f"{f:16s} 连不上：{e}")
            continue
        ans = (r.get("response") or "").strip()
        v = verdict(ans)
        rows.append({
            "file": f,
            "verdict": v,
            "answer": ans.replace("\n", " "),
            "seconds": round(dt, 1),
            "prompt_tok": r.get("prompt_eval_count", 0),
            "out_tok": r.get("eval_count", 0),
        })
        mark = {"有": "●", "没有": "○", "矛盾": "⚠", "?": "?"}[v]
        print(f"{mark} {f:16s} {dt:5.1f}s  p{r.get('prompt_eval_count',0):4d} o{r.get('eval_count',0):3d}  "
              f"[{v}] {ans[:70]}")

    print("\n" + "=" * 70)
    cnt = {}
    for r in rows:
        cnt[r["verdict"]] = cnt.get(r["verdict"], 0) + 1
    print("统计：", "  ".join(f"{k}={v}" for k, v in cnt.items()))

    if out_csv:
        with open(out_csv, "w", newline="", encoding="utf-8-sig") as fp:
            w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("已写：", out_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
