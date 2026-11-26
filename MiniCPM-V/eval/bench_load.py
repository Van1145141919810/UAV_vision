# -*- coding: utf-8 -*-
"""bench_load.py —— 模型加载时间 / 内存占用 / 结果确定性（文档 §6.2 要求项）

═══════════════════════════════════════════════════════════════════
测三件事
───────────────────────────────────────────────────────────────────
1. **冷启动**：模型不在内存时，第一次调用的耗时（含加载）
   —— 决定"部署时第一帧要等多久"，文档 §6.2 明确要求"模型加载时间"
2. **热态**：模型已加载后的单次耗时，与冷启动对比
3. **确定性**：同一张图连问 3 次，答案是否完全一致
   —— temperature=0 是否真的稳定；不稳定就说明结果不可复现

4. **内存**：ollama 进程的峰值工作集（文档要求"资源占用"）

做法：`ollama stop <模型>` 卸载 → 计时第一次调用 → 再连测若干次。

用法：python eval/bench_load.py
"""
import base64
import json
import statistics as st
import subprocess
import sys
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"
IMG = ROOT / "eval" / "crops" / "win08_w1.jpg"
P = ("只看画面正中心的主要物体。它是窗户吗？"
     "只回答一个字母：A=是窗户，B=不是窗户。")


def ask(img_b64, timeout=180):
    body = json.dumps({"model": MODEL, "prompt": P, "images": [img_b64],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout) as r:
        d = json.loads(r.read())
    return d.get("response", "").strip(), time.time() - t0


def mem_mb():
    """所有 ollama / llama 相关进程的工作集合合计（MB）。

    坑：真正的模型跑在 llama-server 子进程里（实测 ~895 MB），
    只统计 ollama.exe 会得到 85 MB 这种离谱数字。
    """
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "(Get-Process | Where-Object {$_.ProcessName -match 'ollama|llama'} | "
             "Measure-Object WorkingSet64 -Sum).Sum"],
            capture_output=True, text=True, timeout=25).stdout.strip()
        return round(int(float(out)) / 1e6, 0) if out else None
    except Exception:
        return None


def main():
    if not IMG.exists():
        raise SystemExit(f"缺测试图 {IMG}")
    b64 = base64.b64encode(IMG.read_bytes()).decode()

    print("卸载模型（模拟冷启动）…")
    subprocess.run(["ollama", "stop", MODEL], capture_output=True, timeout=60)
    time.sleep(3)
    m0 = mem_mb()

    print("第 1 次调用（含加载）…")
    ans1, t_cold = ask(b64)
    m1 = mem_mb()

    # ★ 热态必须换图：同一张图重复请求会命中 Ollama 的 prompt 缓存，
    #   实测能降到 0.08 s，那是缓存不是推理。
    others = sorted((ROOT / "eval" / "crops").glob("*.jpg"))
    warm = []
    for i in range(4):
        p2 = others[(i * 3 + 5) % len(others)]
        a, t = ask(base64.b64encode(p2.read_bytes()).decode())
        warm.append(t)
        print(f"  热态第{i+1}次 {t:.2f}s  answer={a[:12]}")

    print("确定性：同图连问 3 次…")
    det = [ask(b64)[0] for _ in range(3)]

    out = {
        "model": MODEL,
        "cold_start_s": round(t_cold, 2),
        "warm_s": {"mean": round(st.mean(warm), 2),
                   "min": round(min(warm), 2),
                   "max": round(max(warm), 2)},
        "memory_mb_before": m0,
        "memory_mb_after": m1,
        "determinism": det,
        "deterministic": len(set(det)) == 1,
        "answer_cold": ans1[:20],
        "stamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
    }
    f = ROOT / "eval" / "results" / f"load_{out['stamp']}.json"
    f.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n=== 结果 ===")
    print(f"冷启动(含加载) : {out['cold_start_s']:.2f} s")
    print(f"热态均值       : {out['warm_s']['mean']:.2f} s "
          f"({out['warm_s']['min']:.2f}~{out['warm_s']['max']:.2f})")
    print(f"内存(加载前/后): {m0} MB / {m1} MB")
    print(f"确定性         : {'一致' if out['deterministic'] else '不一致 '+str(det)}")
    print(f"明细 {f}")


if __name__ == "__main__":
    main()
