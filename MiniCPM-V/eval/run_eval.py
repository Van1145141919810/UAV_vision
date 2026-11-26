# -*- coding: utf-8 -*-
"""run_eval.py —— MiniCPM-V 离线评测台（岗位 A 的核心交付物）

═══════════════════════════════════════════════════════════════════
它回答文档 §1.3 那几个问题
───────────────────────────────────────────────────────────────────
  1. 真实建筑图像在指定硬件上能否稳定输入视觉模型？
  2. 通用 VLM 对本场景的能力和失败边界是什么？
  3. 推理时延、显存/内存、长期稳定性如何？
  4. 输出是否可追溯、可复现？

═══════════════════════════════════════════════════════════════════
输入
───────────────────────────────────────────────────────────────────
  eval/labels.csv      人工标注（唯一真值来源，必须人工确认）
  data/samples/*.jpg   授权图像

labels.csv 字段：
  image_id, file, scenario, target, gt_is_window, gt_window_state, gt_glare, note

  scenario      场景标签，用于分层统计：outdoor/indoor/night/far/near/occluded...
  target        这张图里有没有"我们的目标"：dot / window / none
  gt_is_window  1/0/空(不适用)   —— 模型问的是"正中是不是窗户"
  gt_window_state  open/closed/n/a
  gt_glare       yes/no/unknown/n/a

═══════════════════════════════════════════════════════════════════
输出（全部可追溯）
───────────────────────────────────────────────────────────────────
  eval/results/<stamp>.csv     每张图一行：原始回答、解析结果、耗时、tokens
  eval/results/<stamp>.json    汇总：准确率/混淆矩阵/分层/时延分布/资源
  eval/report_<stamp>.md       直接可交的报告草稿

═══════════════════════════════════════════════════════════════════
用法
───────────────────────────────────────────────────────────────────
  python eval/run_eval.py                 # 跑全部
  python eval/run_eval.py --reps 3        # 每张重复 3 次（测稳定性）
  python eval/run_eval.py --only win01.jpg,neg1.jpg
  python eval/run_eval.py --no-vlm        # 只跑经典CV基线，做对照
"""
import argparse
import base64
import csv
import json
import statistics
import subprocess
import sys
import time
import urllib.request
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

EVAL = ROOT / "eval"
LABELS = EVAL / "labels.csv"
RESULTS = EVAL / "results"
SAMPLES = ROOT / "data" / "samples"

OLLAMA = "http://127.0.0.1:11434/api/generate"
MODEL = "openbmb/minicpm-v4.7:1b"

# ── 提示词：闭集单字母（我们实测最稳的形式；开放问答会乱答）──────
# 提示词统一从 src/mcpv/prompts.py 取，避免"演示说的"和"评测说的"不一致
from mcpv.prompts import (P_IS_WINDOW as P_GATE,
                          P_WINDOW_STATE as P_OPEN,
                          P_GLARE,
                          PROMPT_VERSION)
# 开放描述：不给选项，让模型自由说，再用关键词判定
P_OPEN_DESC = "用不超过六个字描述画面正中心的主要物体。"
WIN_WORDS = ("窗", "玻璃")          # 出现这些词就判为窗户


# ══════════════════════════════════════════════════════════════
# 模型调用
# ══════════════════════════════════════════════════════════════
def img_b64(p: Path) -> str:
    return base64.b64encode(p.read_bytes()).decode()


def ask(p: Path, prompt: str, timeout=120):
    """调一次 Ollama，返回 (回答, 总耗时, 模型报告的 token 数)。"""
    body = json.dumps({
        "model": MODEL,
        "prompt": prompt,
        "images": [img_b64(p)],
        "stream": False,
        "think": False,
        "keep_alive": "30m",
        "options": {"temperature": 0},
    }).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout) as r:
        d = json.loads(r.read())
    dt = time.time() - t0
    return (d.get("response", "").strip(),
            dt,
            d.get("prompt_eval_count", 0),
            d.get("eval_count", 0))


def letter(answer: str, allowed="ABC"):
    """从回答里取第一个合法字母（模型常带 'A：是窗户' 这种尾巴）。"""
    for ch in answer.upper():
        if ch in allowed:
            return ch
    return "?"


# ══════════════════════════════════════════════════════════════
# 经典 CV 基线（对照组：证明 VLM 到底有没有增益）
# ══════════════════════════════════════════════════════════════
def cv_baseline(p: Path):
    """用我们的颜色+几何方案判"这张图里有没有红色圆形目标"。
    只看 dot 类，作为"有没有 VLM 都一样能做的事"的对照。"""
    try:
        from mcpv.cv import imread_unicode, find_candidates
        img = imread_unicode(p)
        cands = find_candidates(img)
        return len(cands) > 0, len(cands)
    except Exception as e:
        return None, str(e)[:60]


# ══════════════════════════════════════════════════════════════
# 指标
# ══════════════════════════════════════════════════════════════
def pct(x, n):
    return f"{100.0 * x / n:.1f}%" if n else "n/a"


def summarize(rows):
    """算准确率、混淆矩阵、分层、时延分布。"""
    out = {}

    # --- 窗户判定（A/B）---
    sub = [r for r in rows if r["gt_is_window"] in ("0", "1") and r["pred_is_window"] != "?"]
    n = len(sub)
    ok = sum(1 for r in sub if r["pred_is_window"] == r["gt_is_window"])
    tp = sum(1 for r in sub if r["gt_is_window"] == "1" and r["pred_is_window"] == "1")
    fp = sum(1 for r in sub if r["gt_is_window"] == "0" and r["pred_is_window"] == "1")
    fn = sum(1 for r in sub if r["gt_is_window"] == "1" and r["pred_is_window"] == "0")
    tn = sum(1 for r in sub if r["gt_is_window"] == "0" and r["pred_is_window"] == "0")
    out["is_window"] = {
        "n": n, "correct": ok, "acc": pct(ok, n),
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "false_positive_rate": pct(fp, fp + tn),      # 误报（把不是窗户说成窗户）
        "false_negative_rate": pct(fn, fn + tp),      # 漏报（窗户没认出来）
        "recall": pct(tp, tp + fn),
        "precision": pct(tp, tp + fp) if (tp + fp) else "n/a",
    }

    # --- 开合状态 ---
    sub = [r for r in rows if r["gt_window_state"] in ("open", "closed")
           and r["pred_window_state"] in "AB"]
    n = len(sub)
    ok = sum(1 for r in sub if
             (r["pred_window_state"] == "A") == (r["gt_window_state"] == "open"))
    out["window_state"] = {"n": n, "correct": ok, "acc": pct(ok, n)}

    # --- 反光（注意：我们没有可靠真值，只报分布）---
    gl = [r for r in rows if r["pred_glare"] in "AB"]
    out["glare"] = {
        "n": len(gl),
        "said_A_有": sum(1 for r in gl if r["pred_glare"] == "A"),
        "said_B_没有": sum(1 for r in gl if r["pred_glare"] == "B"),
        "note": "无人工真值，只能看分布是否有区分度",
    }

    # --- 时延 ---
    lat = [r["latency_s"] for r in rows if r.get("latency_s")]
    if lat:
        lat_sorted = sorted(lat)
        out["latency_s"] = {
            "n": len(lat),
            "mean": round(statistics.mean(lat), 2),
            "p50": round(lat_sorted[len(lat_sorted) // 2], 2),
            "p95": round(lat_sorted[int(len(lat_sorted) * 0.95) - 1], 2),
            "min": round(min(lat), 2),
            "max": round(max(lat), 2),
        }

    # --- 分层（按 scenario 分组）---
    scen = {}
    for r in rows:
        s = r["scenario"] or "unlabeled"
        d = scen.setdefault(s, {"n": 0, "win_ok": 0, "win_n": 0, "lat": []})
        d["n"] += 1
        if r["gt_is_window"] in ("0", "1") and r["pred_is_window"] != "?":
            d["win_n"] += 1
            if r["pred_is_window"] == r["gt_is_window"]:
                d["win_ok"] += 1
        if r.get("latency_s"):
            d["lat"].append(r["latency_s"])
    for s, d in scen.items():
        d["win_acc"] = pct(d["win_ok"], d["win_n"])
        d["lat_mean"] = round(statistics.mean(d["lat"]), 2) if d["lat"] else None
        d.pop("lat")
    out["by_scenario"] = scen

    # --- 解析失败（模型没按要求回答）---
    out["parse_fail"] = sum(1 for r in rows if "?" in
                            (r["pred_is_window"], r["pred_window_state"]))
    out["total_rows"] = len(rows)
    return out


# ══════════════════════════════════════════════════════════════
# 主流程
# ══════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=1, help="每张图重复次数")
    ap.add_argument("--only", default="", help="逗号分隔的文件名")
    ap.add_argument("--no-vlm", action="store_true", help="只跑 CV 基线")
    ap.add_argument("--labels", default="", help="别的标注表（如 labels_cmp.csv）")
    ap.add_argument("--mode", choices=["ab", "open"], default="ab",
                    help="ab=闭集二选一（默认）；open=开放描述+关键词判定")
    args = ap.parse_args()

    labels_path = Path(args.labels) if args.labels else LABELS
    if not labels_path.exists():
        print(f"✗ 缺标注文件 {labels_path}")
        print("  先跑 python eval/make_labels.py 生成模板，人工填完再评测")
        return 2

    with open(labels_path, encoding="utf-8-sig") as f:
        labels = list(csv.DictReader(f))
    only = [s.strip() for s in args.only.split(",") if s.strip()]
    if only:
        labels = [r for r in labels if Path(r["crop"]).name in only
                  or r["image_id"] in only]

    RESULTS.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rows = []

    print(f"模型 {MODEL}   样本 {len(labels)} 张   reps={args.reps}")
    print(f"{'file':<14}{'gate':<6}{'open':<6}{'glare':<7}{'耗时(s)':<9}{'tokens':<10}")
    print("-" * 62)

    for lb in labels:
        # labels.csv 里 crop 是相对 eval/ 的路径（裁剪在 crops/，整图在 ../data/samples/）
        p = (EVAL / lb["crop"]).resolve()
        if not p.exists():
            print(f"  ✗ 缺图 {p}")
            continue
        for rep in range(args.reps):
            rec = dict(lb, rep=rep, model=MODEL, ts=datetime.now().isoformat(timespec="seconds"))
            if args.no_vlm:
                has, cnt = cv_baseline(p)
                rec.update(latency_s=None, pred_is_window="?", pred_window_state="?",
                           pred_glare="?", ans_gate="", ans_open="", ans_glare="",
                           cv_has_target=has, cv_count=cnt)
                rows.append(rec)
                continue

            if args.mode == "open":
                a1, dt1, pt1, gen1 = ask(p, P_OPEN_DESC)
                a2 = a3 = ""
                dt2 = dt3 = 0.0
                pt2 = pt3 = gen2 = gen3 = 0
            else:
                a1, dt1, pt1, gen1 = ask(p, P_GATE)
                a2, dt2, pt2, gen2 = ask(p, P_OPEN)
                a3, dt3, pt3, gen3 = ask(p, P_GLARE)
            rec.update(
                ans_gate=a1, ans_open=a2, ans_glare=a3,
                pred_is_window=(
                    ("1" if any(w in a1 for w in WIN_WORDS) else "0")
                    if args.mode == "open"
                    else {"A": "1", "B": "0"}.get(letter(a1, "AB"), "?")),
                pred_window_state=("?" if args.mode == "open" else letter(a2, "ABC")),
                pred_glare=("?" if args.mode == "open" else letter(a3, "AB")),
                latency_s=round(dt1 + dt2 + dt3, 2),
                latency_gate=round(dt1, 2),
                tokens_prompt=pt1 + pt2 + pt3,
                tokens_gen=gen1 + gen2 + gen3,
            )
            rows.append(rec)
            print(f"{Path(lb['crop']).stem:<16}{rec['pred_is_window']:<6}{rec['pred_window_state']:<6}"
                  f"{rec['pred_glare']:<7}{rec['latency_s']:<9}"
                  f"{rec['tokens_prompt']}/{rec['tokens_gen']}")

    suffix = "_open" if args.mode == "open" else ""
    csv_path = RESULTS / f"{stamp}{suffix}.csv"
    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    summary = summarize(rows)
    summary.update(model=MODEL, stamp=stamp + suffix, reps=args.reps,
                   mode=args.mode,
                   labels_file=str(LABELS.name))
    json_path = RESULTS / f"{stamp}{suffix}.json"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2),
                         encoding="utf-8")

    print("\n" + "=" * 62)
    print(json.dumps({k: v for k, v in summary.items()
                      if k in ("is_window", "window_state", "glare", "latency_s")},
                     ensure_ascii=False, indent=2))
    print(f"\n明细 {csv_path}")
    print(f"汇总 {json_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
