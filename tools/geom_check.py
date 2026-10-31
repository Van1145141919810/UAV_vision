# -*- coding: utf-8 -*-
"""
geom_check.py —— 靶面坐标三方一致性 (INTERFACE_gate §0): Python vb09.common.geom ↔ 网页 gateAxes / toPlane
  python3 tools/geom_check.py        (需要 node)
从网页源码里抠出 V3 / gateAxes / toPlane 原样交给 node 跑, 和 Python 比。
JS 用 Math.hypot 求模 (末位可能差 1 ulp) → 容差 1e-12。
已知差异: n 竖直 (水平分量为 0) 时 Python right = (0,0,0), 网页退回 [0,-1,0] —— 只打印, 不算失败
(这种法向在穿环里不会出现: 判定 view 角 / 规划都会先拒掉)。
"""
import json
import math
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from vb09.common import geom  # noqa: E402


def js_source():
    for p in (os.path.join(ROOT, "vb09", "web", "kinds", "ring.js"), os.path.join(ROOT, "vb09", "web", "core.js")):
        if os.path.exists(p):
            s = open(p, encoding="utf-8").read()
            if "function gateAxes" in s:
                break
    out = []
    m = re.search(r"const V3 = \{.*?\n\};", s, re.S)
    out.append(m.group(0))
    for fn in ("gateAxes", "toPlane"):
        m = re.search(r"function %s\(.*?\n\}" % fn, s, re.S)
        out.append(m.group(0))
    return "\n".join(out), os.path.relpath(p, ROOT)


CASES = []
for n in ([-1, 0, 0], [-1, -0.12, 0], [-0.7, 0.7, 0.1], [0.3, -0.9, -0.2], [0, 1, 0.05], [0.2, 0.1, 0.97]):
    nn = geom.vunit(n)
    for p in ([0, 0, 1.5], [3.1, -0.4, 1.2], [6.0, 0.3, 1.4], [-2, 5, 0.3]):
        CASES.append((list(nn), p, [5.0, 0.3, 1.4]))
CASES.append(([0.0, 0.0, 1.0], [1, 2, 3], [0, 0, 0]))          # 竖直 n: 已知差异


def main():
    src, where = js_source()
    js = src + """
const cases = %s;
const out = cases.map(([n, p, c]) => { const a = gateAxes(n); const g = {c, n, right: a.right, up: a.up};
  const q = toPlane(g, p); return [a.right, a.up, [q.h, q.v, q.along]]; });
console.log(JSON.stringify(out));""" % json.dumps(CASES)
    r = subprocess.run(["node", "-e", js], capture_output=True, text=True)
    if r.returncode:
        print(r.stderr)
        return 1
    res = json.loads(r.stdout)
    worst, bad, known = 0.0, 0, 0
    for (n, p, c), (jr, ju, jq) in zip(CASES, res):
        pr, pu = geom.gate_axes(n)
        pq = geom.to_plane(p, c, n)
        if math.hypot(n[0], n[1]) < 1e-9:
            known += 1
            print("  known diff (vertical n): py right=%s js right=%s" % (pr, jr))
            continue
        d = max(abs(a - b) for a, b in zip(list(pr) + list(pu) + list(pq), jr + ju + jq))
        worst = max(worst, d)
        bad += d > 1e-12
    print("geom_check  %s  %d cases vs %s  max |Δ| = %.2e  (vertical-n known diffs: %d)"
          % ("PASS" if not bad else "FAIL", len(CASES), where, worst, known))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
