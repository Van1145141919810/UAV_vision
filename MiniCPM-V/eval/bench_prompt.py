# -*- coding: utf-8 -*-
"""bench_prompt.py —— 提示词写法对照（评测方法优化）

═══════════════════════════════════════════════════════════════════
为什么值得做
───────────────────────────────────────────────────────────────────
同一个模型、同一批图，**换一种问法，准确率可能差十几个点**。
文档 §3.1 要求"输出是否可追溯"，而提示词就是最底层的一条"可追溯性"：
不知道用哪句话问出来的结果，就无法复现。

这里固定图和真值，只换提示词，看两件事：
  1. 准确率变不变（会不会把 A/B 的含义换了就翻车）
  2. 有没有"选项位置偏好"（总是选第一个字母）

═══════════════════════════════════════════════════════════════════
四个变体
───────────────────────────────────────────────────────────────────
  v1_AB       A=是窗户 / B=不是窗户        （我们主评测用的）
  v2_BA       A=不是窗户 / B=是窗户        （交换含义 → 查位置偏好）
  v3_yesno    只回答 是 / 不是              （换语义载体）
  v4_open     开放式：它是什么？            （不限制选项，看会不会跑题）

★ v2 是关键：如果 v2 的准确率明显低于 v1，说明模型答的是"位置"而不是"内容"。

用法：python eval/bench_prompt.py
"""
import base64
import csv
import json
import sys
import urllib.request
from collections import defaultdict
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
EVAL = ROOT / "eval"
MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"

VARIANTS = {
    "v1_AB": ("只看画面正中心的主要物体。它是窗户吗？"
              "只回答一个字母：A=是窗户，B=不是窗户。", "AB"),
    "v2_BA": ("只看画面正中心的主要物体。它是窗户吗？"
              "只回答一个字母：A=不是窗户，B=是窗户。", "AB"),
    "v3_yesno": ("只看画面正中心的主要物体。它是窗户吗？只回答两个字：是 或 不是。", None),
    "v4_open": ("用不超过六个字描述画面正中心的主要物体。", None),
}

# v2 里 B 才是窗户；v3/v4 需要额外解析
MEANING = {"v1_AB": {"A": "1", "B": "0"},
           "v2_BA": {"A": "0", "B": "1"}}


def ask(p: Path, prompt: str):
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(p.read_bytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read()).get("response", "").strip()


def parse(variant, ans):
    a = ans.strip()
    if variant in ("v1_AB", "v2_BA"):
        L = next((c for c in a.upper() if c in "AB"), "?")
        return MEANING[variant].get(L, "?"), L
    if variant == "v3_yesno":
        if a.startswith("不是") or "不是" in a[:4]:
            return "0", "不是"
        if a.startswith("是") or "是" in a[:2]:
            return "1", "是"
        return "?", a[:6]
    # v4 开放：用关键词粗判（只做定性展示，不算准确率）
    if any(k in a for k in ("窗", "玻璃", "window")):
        return "1", a[:8]
    return "0", a[:8]


def main():
    with open(EVAL / "labels.csv", encoding="utf-8-sig") as f:
        rows = [r for r in csv.DictReader(f) if r["gt_is_window"] in ("0", "1")]
    print(f"样本 {len(rows)} 条 × {len(VARIANTS)} 个变体 = {len(rows)*len(VARIANTS)} 次调用")

    stat = {v: {"ok": 0, "n": 0, "pos_A": 0} for v in VARIANTS}
    detail = []

    for r in rows:
        p = (EVAL / r["crop"]).resolve()
        for v, (prompt, _) in VARIANTS.items():
            ans = ask(p, prompt)
            pred, raw = parse(v, ans)
            if v != "v4_open" and pred in ("0", "1"):
                stat[v]["n"] += 1
                stat[v]["ok"] += int(pred == r["gt_is_window"])
                stat[v]["pos_A"] += int(raw == "A")
            detail.append({"image": r["image_id"], "gt": r["gt_is_window"],
                           "variant": v, "answer": ans[:24], "pred": pred})
        print(f"  {r['image_id']:<12} 完成", flush=True)

    print(f"\n{'变体':<12}{'准确率':<10}{'n':<6}{'选A次数'}")
    print("-" * 40)
    out = []
    for v in VARIANTS:
        d = stat[v]
        acc = 100.0 * d["ok"] / d["n"] if d["n"] else 0
        out.append({"variant": v, "acc": round(acc, 1), "n": d["n"],
                    "chose_A": d["pos_A"]})
        print(f"{v:<12}{acc:5.1f}%   {d['n']:<6}{d['pos_A']}")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    f = EVAL / "results" / f"prompt_{stamp}.json"
    f.write_text(json.dumps({"model": MODEL, "summary": out, "detail": detail},
                            ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n明细 {f}")


if __name__ == "__main__":
    main()
