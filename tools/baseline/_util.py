# -*- coding: utf-8 -*-
"""基线脚本公用: 规范化序列化 (float 用 repr) + 写/比 golden。"""
import json
import math
import os
import sys

import _compat


def canon(x, _depth=0):
    """任意对象 → 只含 dict/list/str/int/bool/None 的结构; float → repr 字符串 (逐位比较)。"""
    if _depth > 12:
        return "<deep>"
    try:
        import numpy as np
        if isinstance(x, np.ndarray):
            return [canon(v, _depth + 1) for v in x.tolist()]
        if isinstance(x, np.generic):
            x = x.item()
    except ImportError:
        pass
    if x is None or isinstance(x, (bool, str)):
        return x
    if isinstance(x, int):
        return x
    if isinstance(x, float):
        if math.isnan(x):
            return "nan"
        return repr(x)
    if isinstance(x, dict):
        return {str(k): canon(v, _depth + 1) for k, v in sorted(x.items(), key=lambda kv: str(kv[0]))}
    if isinstance(x, (list, tuple, set, frozenset)):
        xs = sorted(x, key=repr) if isinstance(x, (set, frozenset)) else x
        return [canon(v, _depth + 1) for v in xs]
    if hasattr(x, "__dict__"):
        d = {"__cls": type(x).__name__}
        d.update({k: canon(v, _depth + 1) for k, v in sorted(vars(x).items())
                  if not k.startswith("_") and not callable(v)})
        return d
    return repr(x)


def dumps(obj):
    return json.dumps(canon(obj), ensure_ascii=False, indent=1, sort_keys=True)


def golden_path(name):
    os.makedirs(_compat.GOLDEN, exist_ok=True)
    return os.path.join(_compat.GOLDEN, name)


def save_or_compare(name, obj, write, drop=None):
    """
    write=True → 存 golden。否则与 golden 比较: 返回 (ok, 说明)。
    drop(golden_obj, new_obj) → (g, n): 比较前各自去掉白名单字段 (删除戳点那一步用)。
    """
    path = golden_path(name)
    text = dumps(obj)
    if write:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return True, "written %s (%d bytes)" % (os.path.relpath(path, _compat.ROOT), len(text))
    if not os.path.exists(path):
        return False, "no golden %s (run with --write on the archive version first)" % name
    with open(path, encoding="utf-8") as f:
        g = json.load(f)
    n = json.loads(text)
    if drop is not None:
        g, n = drop(g, n)
    if g == n:
        return True, "identical"
    return False, first_diff(g, n)


def first_diff(a, b, path="$", out=None, limit=8):
    out = [] if out is None else out
    if len(out) >= limit:
        return out
    if type(a) != type(b):
        out.append("%s: type %s → %s  (%s → %s)" % (path, type(a).__name__, type(b).__name__,
                                                  _short(a), _short(b)))
    elif isinstance(a, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a:
                out.append("%s.%s: added (%s)" % (path, k, _short(b[k])))
            elif k not in b:
                out.append("%s.%s: removed (%s)" % (path, k, _short(a[k])))
            else:
                first_diff(a[k], b[k], "%s.%s" % (path, k), out, limit)
            if len(out) >= limit:
                break
    elif isinstance(a, list):
        if len(a) != len(b):
            out.append("%s: len %d → %d" % (path, len(a), len(b)))
        for i, (x, y) in enumerate(zip(a, b)):
            first_diff(x, y, "%s[%d]" % (path, i), out, limit)
            if len(out) >= limit:
                break
    elif a != b:
        out.append("%s: %s → %s" % (path, _short(a), _short(b)))
    return out


def _short(x, n=90):
    s = json.dumps(x, ensure_ascii=False)
    return s if len(s) <= n else s[:n] + "…"


def strip_keys(obj, keys):
    """递归删掉名字在 keys 里的字段。"""
    if isinstance(obj, dict):
        return {k: strip_keys(v, keys) for k, v in obj.items() if k not in keys}
    if isinstance(obj, list):
        return [strip_keys(v, keys) for v in obj]
    return obj


def main_args():
    write = "--write" in sys.argv
    return write


def report(name, ok, msg):
    if isinstance(msg, list):
        msg = "\n      ".join(msg)
    print("%-14s %s  %s" % (name, "PASS" if ok else "FAIL", msg))
    return ok
