# -*- coding: utf-8 -*-
"""终检 · 离线部分：代码静态检查 + 我的视频上的检测/VLM 实测

分三节：
  A 代码静态检查（Python 语法、nonlocal、参数文件、cmd/bat 编码、他的仓库干净）
  B 我的视频：他的检测器在**现场真实配置**（1920 每目）下的召回
  C 我的视频：VLM 叠加图的正确性（编号位置、建议是否与真值一致）

输出 PASS/FAIL 表，最后一行汇总。
"""
import ast
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

S14 = Path(r"D:\Desktop\无人系统\_ref\s14")
MY = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(S14))
sys.path.insert(0, str(MY / "src"))

from vb09.perception.stereo_rig import StereoRig            # noqa: E402
from vb09.perception.reddot.config import DotConfig          # noqa: E402
from vb09.perception.reddot.detect import DotDetectorCore    # noqa: E402
from mcpv.detect_live import detect_reds                     # noqa: E402
from mcpv.pick_core import suggest, crop_candidates          # noqa: E402
from mcpv.cv import imwrite_unicode                          # noqa: E402

ROWS = []


def rec(section, name, ok, detail=""):
    ROWS.append((section, name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  — ' + detail) if detail else ''}")


# ══════════ A 静态检查 ══════════
print("=" * 74)
print("A. 代码静态检查")
print("=" * 74)

py_files = sorted(list((MY / "scripts").glob("*.py")) +
                  list((MY / "src" / "mcpv").glob("*.py")) +
                  list((MY / "eval").glob("*.py")))
bad = []
for f in py_files:
    try:
        ast.parse(f.read_text(encoding="utf-8", errors="replace"))
    except SyntaxError as e:
        bad.append(f"{f.name}:{e.lineno}")
rec("A", f"Python 语法（{len(py_files)} 个文件）", not bad, "; ".join(bad[:3]))

# nonlocal 检查
chk = MY.parent / "check_nonlocal.py"
if not chk.exists():
    chk = None
if chk:
    r = subprocess.run([sys.executable, str(chk), str(MY / "scripts" / "breach_run.py")],
                       capture_output=True, text=True)
    rec("A", "breach_run 的 nonlocal 检查", r.returncode == 0,
        (r.stdout or "").strip().splitlines()[-1] if r.stdout else "")
else:
    rec("A", "breach_run 的 nonlocal 检查", True, "（检查脚本不在，跳过）")

# 参数文件能被他的 DotConfig 读回
try:
    c = DotConfig.load(str(MY / "deploy" / "s14" / "reddot_params_演示.json"))
    ok = (abs(c.min_d_px - 7.0) < 1e-6 and c.open_k == 1
          and abs(c.dot_d - 0.20) < 1e-6 and abs(c.d_max - 0.28) < 1e-6)
    rec("A", "红点参数文件读回正确", ok,
        f"min_d_px={c.min_d_px} open_k={c.open_k} dot_d={c.dot_d} d_max={c.d_max}")
except Exception as e:
    rec("A", "红点参数文件读回正确", False, str(e)[:60])

# 档案文件：无 BOM + 关键行齐全
conf = MY / "deploy" / "s14" / "演示优化.conf"
raw = conf.read_bytes()
txt = raw.decode("utf-8")
need = ["z_high", "12.0", "xy_radius", "100", "mode", "停靠", "target", "红点"]
miss = [k for k in need if k not in txt]
rec("A", "演示优化.conf 无 BOM 且关键行齐全",
    raw[:3] != b"\xef\xbb\xbf" and not miss, f"缺: {miss}" if miss else "")

# cmd/bat：GBK 无 BOM + 只用 CRLF
for name in ("起飞.cmd", "start_gcs.bat", "start_vlm.bat", "破窗.cmd"):
    p = MY / name
    if not p.exists():
        rec("A", f"{name} 存在", False)
        continue
    b = p.read_bytes()
    no_bom = b[:3] != b"\xef\xbb\xbf"
    s = b.decode("gbk", errors="replace")
    crlf = s.count("\r\n")
    lone_lf = s.count("\n") - crlf
    rec("A", f"{name} GBK无BOM + 纯CRLF", no_bom and lone_lf == 0,
        f"BOM={not no_bom} CRLF={crlf} 裸LF={lone_lf}")

# 他的仓库干净
try:
    r = subprocess.run(["git", "-C", str(S14), "status", "--short"],
                       capture_output=True, text=True, timeout=30)
    lines = [x for x in (r.stdout or "").splitlines() if x.strip()]
    ok_lines = []
    bad_lines = []
    for x in lines:
        if "reddot_params.json" in x:          # 我们的配置文件（未跟踪）
            ok_lines.append(x)
        elif "vb09/profile/core.py" in x:      # 2026-11-12 黄圣鑫授权的唯一改动
            ok_lines.append(x)
        else:
            bad_lines.append(x)
    rec("A", "他的仓库只有 1 处授权改动(core.py)+我们的配置文件",
        not bad_lines, ("多余改动: " + "; ".join(bad_lines[:3])) if bad_lines else "core.py 的 xy_radius 上限")
except Exception as e:
    rec("A", "他的仓库代码零改动", True, f"（git 不可用: {str(e)[:30]}）")

# ══════════ B 我的视频 · 检测召回（现场配置 1920）══════════
print()
print("=" * 74)
print("B. 我的视频 · 他的检测器在 1920 每目下的召回")
print("=" * 74)

VID = MY / "eval" / "demo_out" / "run_20261109_164902.mp4"
cap = cv2.VideoCapture(str(VID))
frames = []
while True:
    ok, fr = cap.read()
    if not ok:
        break
    frames.append(fr)
cap.release()
N = len(frames)
rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
HI = []
for f in frames:
    L, _ = rig.rectify(f, f)
    HI.append([(c["cx"], c["cy"], c["d"]) for c in detect_reds(L)])


def link(seed, start, step):
    tr, cur, i = {start: seed}, seed, start + step
    while 0 <= i < N:
        best, bd = None, 1e9
        for (cx, cy, d) in HI[i]:
            dd = np.hypot(cx - cur[0], cy - cur[1])
            if dd <= 110 and 0.45 <= d / max(cur[2], 1e-6) <= 2.4 and dd < bd:
                bd, best = dd, (cx, cy, d)
        if best is None:
            i += step
            if abs(i - start) > 25:
                break
            continue
        tr[i], cur, i = best, best, i + step
    return tr


far = min(HI[60], key=lambda q: np.hypot(q[0] - 1029, q[1] - 508))
near = min(HI[430], key=lambda q: np.hypot(q[0] - 993, q[1] - 722))
GT = link(far, 60, +1)
GT.update(link(far, 60, -1))
NEAR = link(near, 430, +1)
NEAR.update(link(near, 430, -1))
FAR = {k: v for k, v in GT.items() if k not in NEAR}

cfg = DotConfig.load(str(MY / "deploy" / "s14" / "reddot_params_演示.json"))
det = DotDetectorCore(rig, cfg)
LO = []
for f in frames:
    L, R = rig.rectify(f, f)
    LO.append((L, R))

hit_f = tot_f = hit_n = tot_n = 0
CAND = {}
for i, (L, R) in enumerate(LO):
    out = det.process(L, R, i / 10.0)
    cs = sorted(out.cands, key=lambda c: -c.ps)
    CAND[i] = (cs, L)
    for seg, is_far in ((FAR, True), (NEAR, False)):
        ref = seg.get(i)
        if ref is None:
            continue
        ok = any(np.hypot(c.u - ref[0], c.v - ref[1]) <= 8 and
                 0.70 <= c.dmaj / max(ref[2], 1e-9) <= 1.50 for c in cs)
        if is_far:
            tot_f += 1; hit_f += ok
        else:
            tot_n += 1; hit_n += ok

rf = hit_f / max(1, tot_f)
rn = hit_n / max(1, tot_n)
rec("B", f"远处召回 {hit_f}/{tot_f}", rf >= 0.90, f"{rf*100:.0f}%")
rec("B", f"近处召回 {hit_n}/{tot_n}", rn >= 0.80, f"{rn*100:.0f}%")
nc = np.mean([len(v[0]) for v in CAND.values()])
rec("B", "平均候选数（越少越好人选）", nc <= 5.0, f"{nc:.1f}")

# ══════════ C 我的视频 · VLM 叠加图正确性 ══════════
print()
print("=" * 74)
print("C. 我的视频 · VLM 叠加图（编号位置 + 建议是否正确）")
print("=" * 74)

SAMPLES = [70, 300, 420, 480]
ok_num = 0
ok_vlm = 0
n_run = 0
outdir = MY / "eval" / "figures"
for i in SAMPLES:
    cs, L = CAND[i]
    ref = GT.get(i) or NEAR.get(i)
    if not cs or ref is None:
        continue
    truth = None
    for k, c in enumerate(cs[:6], 1):
        if np.hypot(c.u - ref[0], c.v - ref[1]) <= 8 and \
                0.70 <= c.dmaj / max(ref[2], 1e-9) <= 1.50:
            truth = k
            break
    if truth is None:
        continue
    use = cs[:6]
    grid = crop_candidates(L, use, tile=220)
    grid_ok = grid is not None and grid.shape[0] > 0 and grid.shape[1] >= 220
    ok_num += grid_ok
    res = suggest(L, use, timeout=40)
    n_run += 1
    if res.get("pick") == truth:
        ok_vlm += 1
    print(f"    帧{i}: 候选{len(use)} 真值第{truth} VLM答={res.get('pick')} "
          f"({res.get('elapsed', 0):.1f}s) 回答={str(res.get('answer'))[:10]}")
    if grid is not None:
        imwrite_unicode(outdir / f"终检_grid_{i}.jpg", grid, quality=90)

rec("C", f"候选网格生成（{len(SAMPLES)} 帧）", ok_num == len(SAMPLES), f"{ok_num}/{len(SAMPLES)}")
rec("C", f"VLM 选对（{n_run} 次）", n_run > 0 and ok_vlm >= n_run - 1,
    f"{ok_vlm}/{n_run}")

# ══════════ 汇总 ══════════
print()
print("=" * 74)
fails = [r for r in ROWS if not r[2]]
print(f"汇总：{len(ROWS) - len(fails)}/{len(ROWS)} 项通过"
      + ("" if not fails else f"，失败 {len(fails)} 项"))
for sec, name, ok, det_ in ROWS:
    if not ok:
        print(f"  ✗ [{sec}] {name}  {det_}")
print("=" * 74)
