# -*- coding: utf-8 -*-
"""
vb09/monitor/logs.py —— 飞行日志 (网页「回放」, 只读)
===============================================================================
2026-10-30 第 5 步从 flight_monitor.py 原样搬来 (tools/dev/split_monitor.py)。
"""

import os

from vb09.monitor.common import LOG_DIR_RE, LOG_FILE_RE, LOG_MAX_BYTES
from vb09.monitor.store import STORE


# =============================================================================
# 飞行日志 (网页「回放」页, 只读)
# =============================================================================
def _live_log_name():
    with STORE.lock:
        live = (STORE.config or {}).get("log_dir") or (STORE.telemetry or {}).get("log_dir")
    return os.path.basename(str(live).rstrip("/")) if live else None


def list_logs(args, limit=80):
    """~/uav_logs 下的飞行目录 (新的在前), 每个目录里能读的 csv/txt/log 文件。"""
    root = os.path.expanduser(args.log_root)
    out = {"root": root, "flights": [], "live": _live_log_name()}
    if not os.path.isdir(root):
        return out
    names = sorted((d for d in os.listdir(root)
                    if LOG_DIR_RE.match(d) and os.path.isdir(os.path.join(root, d))),
                   reverse=True)[:limit]
    for n in names:
        p = os.path.join(root, n)
        files = []
        try:
            for f in sorted(os.listdir(p)):
                fpath = os.path.join(p, f)
                if LOG_FILE_RE.match(f) and os.path.isfile(fpath):
                    files.append({"name": f, "size": os.path.getsize(fpath)})
            out["flights"].append({"name": n, "mtime": os.path.getmtime(p), "files": files})
        except OSError:
            continue
    return out


def read_log(args, d, f):
    """→ (http 状态码, bytes 或 dict)。只认 flight_* 目录下的 csv/txt/log, 不跟出根目录。"""
    root = os.path.realpath(os.path.expanduser(args.log_root))
    if not LOG_DIR_RE.match(d or "") or not LOG_FILE_RE.match(f or ""):
        return 400, {"ok": False, "error": "目录名/文件名不合法"}
    p = os.path.realpath(os.path.join(root, d, f))
    if not p.startswith(root + os.sep) or not os.path.isfile(p):
        return 404, {"ok": False, "error": "没有这个文件"}
    if os.path.getsize(p) > LOG_MAX_BYTES:
        return 413, {"ok": False, "error": "文件太大 (> %d MB), 用 scp 拷出来看"
                     % (LOG_MAX_BYTES // 1024 // 1024)}
    with open(p, "rb") as fh:
        return 200, fh.read()
