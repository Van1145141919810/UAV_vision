# -*- coding: utf-8 -*-
"""一次性 (第 5 步): flight_profile.py → vb09/profile/core.py + gate.py, 节点原样搬, 互相引用加 _c. / _g. 前缀。"""
import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from qualify import qualify  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
src = open(os.path.join(ROOT, "flight_profile.py"), encoding="utf-8").read()
lines = src.splitlines(keepends=True)
t = ast.parse(src)
GATE = set("MODE_NAMES GATE_KNOB_PARAM GATE_SAFETY_RULES _build_gate_overrides _gate_preview _check_gate "
           "_derived_gate _start_line _gate_lines _print_envelope_gate _site_config_text_gate".split())
N_CORE_KNOBS = 7

doc_end = t.body[0].end_lineno
imports_end = max(n.end_lineno for n in t.body if isinstance(n, (ast.Import, ast.ImportFrom)))
core_parts, gate_parts = [], []
prev = imports_end
main_block = None
for n in t.body:
    if n.lineno <= imports_end:
        continue
    text = "".join(lines[prev:n.end_lineno])
    prev = n.end_lineno
    if isinstance(n, ast.If):
        main_block = text
        continue
    nm = n.name if isinstance(n, (ast.FunctionDef, ast.ClassDef)) else n.targets[0].id
    if nm == "KNOBS":
        elts = n.value.elts
        head = "".join(lines[n.lineno - 1 - (n.lineno - 1 - (n.lineno - 1)):n.lineno - 1])  # noqa
        pre = "".join(lines[prev - (n.end_lineno - n.lineno + 1) - (n.lineno - 1 - (prev - (n.end_lineno - n.lineno + 1))):n.lineno - 1]) if False else text[:text.index("KNOBS = [")]
        seg = lambda a, b: "".join(lines[a:b])   # noqa: E731
        core_k = seg(elts[0].lineno - 1, elts[N_CORE_KNOBS - 1].end_lineno)
        gate_k = seg(elts[N_CORE_KNOBS].lineno - 1, elts[-1].end_lineno)
        # 第 N_CORE_KNOBS 个元素前面的注释 (# ---- 穿环 ----) 一起给 gate
        gap = seg(elts[N_CORE_KNOBS - 1].end_lineno, elts[N_CORE_KNOBS].lineno - 1)
        core_parts.append(pre + "CORE_KNOBS = [\n" + core_k + "\n]\n# 穿环旋钮在 vb09/profile/gate.py (GATE_KNOBS), 表顺序 = 共用在前、穿环在后 (和拆分前同一个顺序)\n"
                          "KNOBS = CORE_KNOBS + _g.GATE_KNOBS\n")
        gate_parts.append("\n\nGATE_KNOBS = [\n" + gap.lstrip("\n").rstrip() + "\n" + gate_k + "\n]\n")
        continue
    (gate_parts if nm in GATE else core_parts).append(text)

def names_of(code):
    tr = ast.parse(code)
    out = set()
    for n in tr.body:
        if isinstance(n, (ast.FunctionDef, ast.ClassDef)):
            out.add(n.name)
        elif isinstance(n, ast.Assign):
            out |= {x.id for x in n.targets if isinstance(x, ast.Name)}
    return out

core_code = "".join(core_parts)
gate_code = "".join(gate_parts)
core_defs, gate_defs = names_of(core_code), names_of(gate_code) | {"GATE_KNOBS"}
core_q, n1 = qualify(core_code, {k: k for k in gate_defs}, "_g")
gate_q, n2 = qualify(gate_code, {k: k for k in core_defs}, "_c")
if core_q is None or gate_q is None:
    sys.exit("qualify failed: %s %s" % (n1, n2))
doc = "".join(lines[:doc_end])
imp = "".join(lines[t.body[1].lineno - 1:imports_end]) + "\n"
core = (doc.replace('flight_profile.py —— 现场档案（site profile）：把「现场环境」翻译成飞行限制',
                    'vb09/profile/core.py —— 现场档案（site profile）：把「现场环境」翻译成飞行限制 (通用部分)', 1)
        .replace('"""\n', '"""\n2026-10-30 第 5 步: 原 flight_profile.py 拆成 core (解析/格式化/--set/setup/finalize/JSON) 和\n'
                 'gate (穿环旋钮/校验/派生/打印); 代码原样, 互相引用加 _g. / _c. 前缀。根目录 flight_profile.py 是兼容壳。\n', 1)
        + "\n" + imp + "from vb09.profile import gate as _g\n\n" + core_q)
gate = ('# -*- coding: utf-8 -*-\n"""\nvb09/profile/gate.py —— 现场档案的穿环部分: 穿环旋钮 / 旋钮→参数 / 校验 / 派生量 / 打印\n'
        '===============================================================================\n'
        '2026-10-30 第 5 步从 flight_profile.py 原样搬来; 通用部分 (表结构、_fmt、SCENE_TABLE …) 用 _c. 引 core。\n'
        '★ core 在文件**末尾**才 import: core 导入时要用这里的 GATE_KNOBS, 两边谁先被 import 都能成立。\n"""\n'
        + imp + gate_q + "\n\nfrom vb09.profile import core as _c  # noqa: E402  (见文件头)\n")
os.makedirs(os.path.join(ROOT, "vb09", "profile"), exist_ok=True)
open(os.path.join(ROOT, "vb09", "profile", "core.py"), "w", encoding="utf-8").write(core)
open(os.path.join(ROOT, "vb09", "profile", "gate.py"), "w", encoding="utf-8").write(gate)
open(os.path.join(ROOT, "tools", "dev", "_profile_main.txt"), "w", encoding="utf-8").write(main_block or "")
print("core edits %d, gate edits %d" % (n1, n2))
