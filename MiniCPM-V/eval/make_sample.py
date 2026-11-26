# -*- coding: utf-8 -*-
"""make_sample.py —— 生成给 Jev 的 10 条真实样例（schema_sample.jsonl）

数据来源：bench_filter 的实测记录（分割框 + VLM 描述 + 真值窗占比），
转成 docs/06_给Jev的接口约定.md 里定义的 cpmv.observation/v1 格式。
"""
import json
from datetime import datetime
from pathlib import Path

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
R = ROOT / "eval" / "results"

f = sorted(R.glob("filter_*.json"))[-1]
d = json.loads(f.read_text(encoding="utf-8"))

CATEGORY = [
    ("门", "door"),
    ("空调", "ac_unit"),
    ("广告", "sign"),
    ("玻璃", "glass"),
    ("墙", "wall"),
    ("窗", "window"),
]


def category(desc):
    for k, v in CATEGORY:
        if k in desc:
            return v
    return "unknown"


def band(fr):
    if fr >= 0.60:
        return "high"
    if fr <= 0.05:
        return "low"
    return "mid"


rows = []
for i, r in enumerate(d["records"][:10], 1):
    fr = r["win_frac"]
    rows.append({
        "schema": "cpmv.observation/v1",
        "obs_id": f"{r['image']}_b{i}_{datetime.now():%Y%m%dT%H%M}",
        "ts": datetime.now().astimezone().isoformat(timespec="seconds"),
        "source": {
            "frame": r["image"] + ".jpg",
            "derivation": "segformer_window_component",
        },
        "geometry": {
            "bbox_px": r["box"],
            "win_fraction": fr,
        },
        "semantics": {
            "describe": r["desc"],
            "category": category(r["desc"]),
            "state": "unknown",      # 实测未验证，统一给 unknown
            "glare": "unknown",      # 实测恒答"有"，不给判据
        },
        "quality": {
            "win_fraction_band": band(fr),
            "needs_review": band(fr) == "mid",
            "flags": [] if band(fr) != "mid" else ["ambiguous_region"],
        },
        "runtime": {
            "seg_model": d["seg"],
            "vlm_model": d["vlm"],
            "prompt_version": "v1_20261105",
        },
    })

out = R / "schema_sample.jsonl"
out.write_text("\n".join(json.dumps(x, ensure_ascii=False) for x in rows) + "\n",
               encoding="utf-8")
print(f"写出 {len(rows)} 条 → {out}")
print("\n前两条预览：")
for x in rows[:2]:
    print(json.dumps(x, ensure_ascii=False, indent=2))
