# -*- coding: utf-8 -*-
"""一次性 (第 5 步): flight_monitor.py → vb09/monitor/*.py (节点原样搬, 跨模块名字按用到的自动 import)。
入口 flight_monitor.py 留: Handler (HTTP) / bind_or_die / main。LAUNCHER_TMPL → vb09/monitor/fly.sh.tmpl。"""
import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
P = os.path.join(ROOT, "flight_monitor.py")
src = open(P, encoding="utf-8").read()
lines = src.splitlines(keepends=True)
t = ast.parse(src)

MOD = {
    "common": "SCRIPT_NODE STATUS_TOPIC SETPOINT_PREFIX HOSTNAME TEL_BUF_N GATE_BUF_N STREAM_INIT_TEL_S "
              "STREAM_INIT_GATE_S STREAM_MAX_TEL STREAM_MAX_GATE PX4_PARAMS LOG_DIR_RE LOG_FILE_RE LOG_MAX_BYTES "
              "TASK_LABEL _abs_node SCRIPT_RE SUFFIX_TASK KNOB_TASK discover_scripts task_of_script script_for "
              "_no_script_msg clean att_from_quat",
    "store": "RateMeter Store STORE",
    "checks": "current_task _bridge_mode gate_checks _fmt_param px4_check camera_check run_checks paths_info",
    "launcher": "_preview_lock NAME_RE SET_RE normalize_knobs normalize_sets dry_run script_cmd default_config "
                "save_site TEMP_SITE TEMP_DIR _flying temp_launcher LAUNCHER_TMPL _sha256 _tilde write_launcher "
                "launcher_status regen_launcher",
    "logs": "_live_log_name list_logs read_log",
    "ros": "start_ros",
    "fake": "quat_from_euler FAKE_CAM FAKE_PX4 _va _vs _vm _vd _vx _vu gate_axes cam_R_WC start_fake_gate",
    "_entry": "Handler bind_or_die main",
}
OWNER = {n: m for m, ns in MOD.items() for n in ns.split()}
DOC = {
    "common": "常量、飞行脚本发现 (stageNN_<任务>.py)、clean / att_from_quat 小工具",
    "store": "话题数据汇总 (Store 单例 STORE) + 频率计",
    "checks": "机况检查 (run_checks) + 文件与路径 (paths_info)",
    "launcher": "配置页: dry-run / 写档案 / 生成启动脚本 ~/fly.sh (模板 fly.sh.tmpl)",
    "logs": "飞行日志 (网页「回放」, 只读)",
    "ros": "ROS 订阅 + ROS 图 / PX4 参数轮询 (start_ros)",
    "fake": "离线假数据 (--fake): 一架假飞机看一个假环, 带假识别画面",
}
STD = [("argparse", "import argparse"), ("collections", "import collections"), ("errno", "import errno"),
       ("json", "import json"), ("math", "import math"), ("os", "import os"), ("re", "import re"),
       ("shutil", "import shutil"), ("socket", "import socket"), ("statistics", "import statistics"),
       ("subprocess", "import subprocess"), ("sys", "import sys"), ("tempfile", "import tempfile"),
       ("threading", "import threading"), ("time", "import time"),
       ("ThreadingHTTPServer", "from http.server import ThreadingHTTPServer"),
       ("BaseHTTPRequestHandler", "from http.server import BaseHTTPRequestHandler"),
       ("urlparse", "from urllib.parse import urlparse"), ("parse_qs", "from urllib.parse import parse_qs"),
       ("unquote", "from urllib.parse import unquote"), ("geom", "from vb09.common import geom")]

hdr_end = max(n.end_lineno for n in t.body[:30] if isinstance(n, (ast.Import, ast.ImportFrom)))
print("orig imports:", "".join(lines[t.body[1].lineno - 1:hdr_end + 8]))
STD.append(("datetime", "from datetime import datetime"))
first = next(i for i, n in enumerate(t.body) if isinstance(n, ast.Assign) and n.targets[0].id == "SCRIPT_NODE")
i_main = len(t.body) - 1
assert isinstance(t.body[i_main], ast.If)
prev = t.body[first - 1].end_lineno
per = {m: [] for m in MOD}
for n in t.body[first:i_main]:
    text = "".join(lines[prev:n.end_lineno])
    prev = n.end_lineno
    if isinstance(n, (ast.FunctionDef, ast.ClassDef)):
        nm = [n.name]
    else:
        tg = n.targets[0]
        nm = [x.id for x in ast.walk(tg) if isinstance(x, ast.Name)]
    owners = {OWNER[x] for x in nm}
    assert len(owners) == 1, nm
    m = owners.pop()
    if m == "launcher" and nm == ["LAUNCHER_TMPL"]:
        val = ast.literal_eval(n.value)
        open(os.path.join(ROOT, "vb09", "monitor", "fly.sh.tmpl"), "w", encoding="utf-8", newline="").write(val)
        cm = text[:text.index("LAUNCHER_TMPL = ")]
        text = cm + ('# 2026-10-30: 模板正文搬到同目录 fly.sh.tmpl (原样, 占位符 @@名字@@)\n'
                     'with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "fly.sh.tmpl"),\n'
                     '          encoding="utf-8", newline="") as _f:\n    LAUNCHER_TMPL = _f.read()\n')
    per[m].append(text)
tail = "".join(lines[t.body[i_main - 1].end_lineno:])
defined = {m: set() for m in MOD}
for m, ns in MOD.items():
    defined[m] = set(ns.split())
outs = {}
for m, parts in per.items():
    code = "".join(parts).strip("\n") + "\n"
    tr = ast.parse(code)
    used = {x.id for x in ast.walk(tr) if isinstance(x, ast.Name)}
    imps = [s for k, s in STD if k in used]
    cross = {}
    for u in sorted(used - defined[m]):
        o = OWNER.get(u)
        if o and o != m and o != "_entry":
            cross.setdefault(o, []).append(u)
    loc = []
    if "HERE" in used or "fp" in used:
        if m != "common":
            cross.setdefault("common", [])
            cross["common"] += [x for x in ("HERE", "fp") if x in used]
    for o in sorted(cross):
        loc.append("from vb09.monitor.%s import %s" % (o, ", ".join(sorted(set(cross[o])))))
    if m == "_entry":
        outs[m] = (imps, loc, code)
        continue
    head = ('# -*- coding: utf-8 -*-\n"""\nvb09/monitor/%s.py —— %s\n%s\n2026-10-30 第 5 步从 flight_monitor.py 原样搬来 '
            '(tools/dev/split_monitor.py)。\n"""\n' % (m, DOC[m], "=" * 79))
    pre = ""
    if m == "common":
        pre = ('# 程序根目录 (= flight_monitor.py 所在目录; 飞行脚本 / monitor.html / 档案都按它找)\n'
               'HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))\n'
               'if HERE not in sys.path:\n    sys.path.insert(0, HERE)\ntry:\n    import flight_profile as fp\n'
               'except ImportError:\n    fp = None\n\n')
        for s in ("import os", "import sys"):
            if s not in imps:
                imps.append(s)
    txt = head + "\n" + "\n".join(sorted(set(imps), key=lambda s: (s.startswith("from"), s))) + "\n" + \
        ("\n" + "\n".join(loc) + "\n" if loc else "") + "\n\n" + pre + code
    outs[m] = txt
for m, txt in outs.items():
    if m != "_entry":
        open(os.path.join(ROOT, "vb09", "monitor", m + ".py"), "w", encoding="utf-8").write(txt)
imps, loc, code = outs["_entry"]
# 入口: 原文件头 (docstring + 原 import 段) + 新的 import + Handler/bind_or_die/main + __main__
head_end = t.body[first - 1].end_lineno
entry_head = "".join(lines[:head_end])
entry = entry_head + "\n" + "\n".join(loc) + "\n" + \
    "from vb09.monitor.common import *  # noqa: F401,F403,E402  (老代码 / 测试按 flight_monitor.X 取常量)\n\n\n" + \
    code + tail
open(P, "w", encoding="utf-8").write(entry)
print("ok")
