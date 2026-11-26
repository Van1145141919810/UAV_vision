# -*- coding: utf-8 -*-
"""
bench_matrix.py —— 同一批图，横向比较多种"问法"。

用途：证明"问法比模型影响更大"——同一个模型、同一批图，换问法准确率差很多。
输出：屏幕上的矩阵表 + 一个 csv（放 数据\\ 目录）。

用法：
    python bench_matrix.py 图片目录
"""

import sys
import os
import io
import re
import csv
import json
import time
import base64
import urllib.request
import urllib.error

HOST = "http://127.0.0.1:11434"
MODEL = "openbmb/minicpm-v4.7:1b"

# ---- 真值：文件名 -> 有没有红点（人工逐张核对过）----
TRUTH = {
    "test1.jpg": "无", "test2.jpg": "有", "test3.jpg": "有",
    "neg1.jpg": "无", "neg2.jpg": "无", "neg3.jpg": "无",
    "neg4.jpg": "无", "neg5.jpg": "无",
}

# ---- 待比较的问法 ----
PROMPTS = [
    ("P1单字", "仔细看这张图。图中有没有红色圆点？只回答一个字：有 或 无。"),
    ("P2上下文", "图中是否有贴在窗玻璃或白纸上的红色圆形标记？只回答一个字：有 或 无。"),
    ("P3对称AB", "这张图属于哪一种情况？A：画面中有红色圆形标记。B：画面中没有红色圆形标记。只回答 A 或 B。"),
    ("P4打分", "画面中出现红色圆形标记的可能性有多大？只输出一个 0 到 100 的整数，不要任何其他文字。"),
]

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


def ask(img, prompt, timeout=900):
    raw = open(img, "rb").read()
    payload = {
        "model": MODEL, "prompt": prompt,
        "images": [base64.b64encode(raw).decode("ascii")],
        "stream": False, "options": {"temperature": 0},
        "keep_alive": "30m", "think": False,
    }
    req = urllib.request.Request(
        HOST + "/api/generate",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"})
    t0 = time.time()
    r = json.loads(urllib.request.urlopen(req, timeout=timeout).read())
    return (r.get("response") or "").strip(), time.time() - t0


def parse(kind, ans):
    a = (ans or "").strip()
    head = a[:12]
    if kind == "P3对称AB":
        if re.match(r"^[Aa]", head):
            return "有"
        if re.match(r"^[Bb]", head):
            return "无"
        return "?"
    if kind == "P4打分":
        m = re.search(r"\d{1,3}", head)
        if not m:
            return "?"
        v = int(m.group())
        return f"{v}"        # 数字原样返回，后面按阈值判
    # 单字类
    if head.startswith("无") or head.startswith("没有") or head.startswith("不存在"):
        return "无"
    if head.startswith("有") or head.startswith("存在") or head.startswith("是"):
        return "有"
    if "没有" in head:
        return "无"
    if "有" in head:
        return "有"
    return "?"


def main():
    fix_console()
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    folder = sys.argv[1]
    files = [f for f in sorted(os.listdir(folder))
             if f.lower().endswith(EXTS) and f in TRUTH]
    if not files:
        print("[x] 目录里没有已知真值的图片")
        return 1

    results = {}
    for f in files:
        p = os.path.join(folder, f)
        for name, prompt in PROMPTS:
            ans, dt = ask(p, prompt)
            v = parse(name, ans)
            results[(f, name)] = (v, ans, dt)
            print(f"  {f:10s} {name:9s} {dt:5.1f}s  -> {v:3s}  | {ans[:40]}")

    print("\n" + "=" * 78)
    header = f"{'图片':<12}{'真值':<6}" + "".join(f"{n:<10}" for n, _ in PROMPTS)
    print(header)
    print("-" * 78)
    for f in files:
        row = f"{f:<12}{TRUTH[f]:<6}"
        for name, _ in PROMPTS:
            v = results[(f, name)][0]
            ok = "✓" if v == TRUTH[f] else "✗"
            row += f"{v+ok:<10}"
        print(row)

    # 准确率（P4 按 >=50 算阳性）
    print("-" * 78)
    for name, _ in PROMPTS:
        hit = 0
        for f in files:
            v = results[(f, name)][0]
            if name == "P4打分":
                pred = "有" if (v.isdigit() and int(v) >= 50) else "无"
            else:
                pred = v
            if pred == TRUTH[f]:
                hit += 1
        print(f"{name:10s} 准确率 {hit}/{len(files)} = {hit/len(files)*100:.0f}%")

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "数据", "问法对比_20261102.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["图片", "真值"] + [n for n, _ in PROMPTS])
        for f in files:
            w.writerow([f, TRUTH[f]] + [results[(f, n)][0] for n, _ in PROMPTS])
    print("\n已写：", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
