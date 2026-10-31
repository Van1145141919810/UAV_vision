# -*- coding: utf-8 -*-
"""
vb09/monitor/launcher.py —— 配置页: dry-run / 写档案 / 生成启动脚本 ~/fly.sh (模板 fly.sh.tmpl)
===============================================================================
2026-10-30 第 5 步从 flight_monitor.py 原样搬来 (tools/dev/split_monitor.py)。
"""

import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime

from vb09.monitor.common import _no_script_msg, fp, script_for
from vb09.monitor.store import STORE


# =============================================================================
# 配置: dry-run / 写档案
# =============================================================================
_preview_lock = threading.Lock()
# 档案名: 中文/字母/数字/_ - . 都行 (★2026-10-21 加了 "."; 以前"穿环快速2.5"这种名字在网页里读不出来,
#   表单被静默清空成代码默认值)。不许: 路径分隔符、空白、引号等 shell 特殊字符、以 . 或 - 开头、".."
NAME_RE = re.compile(r"^(?![.\-])(?!.*\.\.)[\w一-鿿\-.]{1,40}$")
SET_RE = re.compile(r"^[A-Z][A-Z0-9_]*=[^\s]+$")


def normalize_knobs(raw):
    """网页表单 → 旋钮 dict。空值 = 不给 (用代码默认值)。"""
    if not isinstance(raw, dict):
        raise ValueError("knobs 必须是对象")
    out = {}
    for k, v in raw.items():
        spec = fp.KNOB_INDEX.get(k)
        if spec is None:
            raise ValueError("未知旋钮 %r" % k)
        if v is None or (isinstance(v, str) and v.strip() == ""):
            continue
        if spec["type"] == "float":
            try:
                f = float(v)
            except (TypeError, ValueError):
                raise ValueError("%s 需要数字, 得到 %r" % (k, v))
            if not math.isfinite(f):
                raise ValueError("%s 需要有限数字, 得到 %r" % (k, v))
            out[k] = f
        else:
            s = str(v).strip()
            if "#" in s or "\n" in s or "=" in s:
                raise ValueError("%s 不能包含 # = 或换行" % k)
            out[k] = s
    return out


def normalize_sets(raw):
    if raw is None:
        return []
    if isinstance(raw, str):
        raw = raw.splitlines()
    out = []
    for line in raw:
        line = str(line).strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("--set"):
            line = line[5:].strip()
        if line.startswith("set."):
            line = line[4:].replace(" ", "")
        if not SET_RE.match(line):
            raise ValueError("--set 行格式应为 KEY=VALUE (大写参数名, 值不含空格): %r"
                             % line)
        out.append(line)
    return out


def dry_run(args, knobs, sets):
    """
    把旋钮 + set 写进临时档案, 跑 <飞行脚本> --dry-run --json。
    set 也写进档案 (而不是拼 --set), 预览走的就是保存后真正起飞时的同一条路径。
    """
    tmp = tempfile.mkdtemp(prefix="fm_preview_")
    try:
        with open(os.path.join(tmp, "__preview__.conf"), "w", encoding="utf-8") as f:
            f.write(fp.format_conf(knobs, header="flight_monitor 预览", sets=sets))
        task, script = script_for(args, knobs)
        if not script or not os.path.exists(script):
            return {"ok": False, "errors": [_no_script_msg(args, task)]}
        cmd = [sys.executable, script, "--sites-dir", tmp,
               "--site", "__preview__", "--dry-run", "--json"]
        with _preview_lock:
            p = subprocess.run(cmd, cwd=os.path.dirname(script),
                               capture_output=True, text=True, timeout=30)
        try:
            res = json.loads(p.stdout)
        except ValueError:
            return {"ok": False, "errors": ["dry-run 没有输出 JSON (退出码 %d)"
                                            % p.returncode],
                    "stderr": (p.stderr or p.stdout)[-3000:]}
        res["stderr"] = (p.stderr or "")[-2000:]
        res["exit_code"] = p.returncode
        res["script"] = os.path.basename(script)
        return res
    except subprocess.TimeoutExpired:
        return {"ok": False, "errors": ["dry-run 超时"]}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def script_cmd(args, tail="", script=None):
    """飞行脚本的启动命令(带 cd, 复制到任何目录下的 SSH 窗口里都能直接跑)。"""
    script = script or args.script
    if not script:
        return "cd ~/<目录> && python3 stageNN_<任务>.py"
    d = os.path.dirname(script)
    home = os.path.expanduser("~")
    if d == home or d.startswith(home + os.sep):
        d = "~" + d[len(home):]
    return ("cd %s && python3 %s %s" % (d, os.path.basename(script), tail)).rstrip()


def default_config(args):
    """代码默认值的 dry-run 结果, 脚本启动前给页面作参考。"""
    if fp is None or not args.script or not os.path.exists(args.script):
        return None
    cmd = [sys.executable, args.script, "--dry-run", "--json",
           "--sites-dir", args.sites_dir]
    try:
        p = subprocess.run(cmd, cwd=os.path.dirname(args.script),
                           capture_output=True, text=True, timeout=30)
        res = json.loads(p.stdout)
    except Exception as e:
        print("[monitor] 取代码默认配置失败: %s" % e)
        return None
    return {"t": time.time(), "src": "代码默认值 (脚本还没启动, 仅供参考)",
            "task": res.get("task"),     # 脚本没跑时网页靠它选布局 (INTERFACE_gate §4)
            "knobs": {}, "lines": res.get("lines", []),
            "warnings": res.get("warnings", []) + res.get("errors", []),
            "derived": res.get("derived"), "params": res.get("params", {}),
            "origin": None, "log_dir": None}


def save_site(args, body):
    name = str(body.get("name", "")).strip()
    if not NAME_RE.match(name) or name.startswith("__"):
        return 400, {"ok": False, "error": "档案名只能用中文/字母/数字/_/-, 1~40 字"}
    s = STORE.snapshot()
    armed = bool(s["mavros"].get("armed")) or bool(
        s["script_alive"] and (s["telemetry"] or {}).get("air"))
    if armed:
        return 409, {"ok": False, "error": "飞机已解锁/在空中 —— 飞行中不写档案"}
    try:
        knobs = normalize_knobs(body.get("knobs") or {})
        sets = normalize_sets(body.get("sets"))
    except ValueError as e:
        return 400, {"ok": False, "error": str(e)}
    res = dry_run(args, knobs, sets)
    if not res.get("ok"):
        return 422, {"ok": False, "error": "校验不通过, 不保存",
                     "errors": res.get("errors", [])}
    d = os.path.expanduser(args.sites_dir)
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, name + ".conf")
    if os.path.exists(path):
        if not body.get("overwrite"):
            return 409, {"ok": False, "error": "档案 %s 已存在 (勾选覆盖, 或换个名字)"
                                               % name, "exists": True}
        shutil.copy2(path, path + ".bak")
    header = "由 flight_monitor 网页生成 %s" % datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(fp.format_conf(knobs, header=header, sets=sets))
    os.replace(tmp, path)
    # set 已经在档案里了 → 启动命令只有 --site, 不会因为少复制一段而变成另一套配置
    script = script_for(args, knobs)[1]
    cmd = script_cmd(args, "--site %s" % name, script)
    STORE.add_event("INFO", "已保存现场档案 %s" % name, "monitor")
    out = {"ok": True, "path": path, "command": cmd, "dry_run_command": cmd + " --dry-run"}
    if body.get("launcher"):
        out["launcher"] = write_launcher(args, name, script=script)
    return 200, out


# ★2026-10-12: 「只生成启动脚本, 不存档案」—— 临时测试用。
#   配置写到档案目录**外面**的一个固定位置 (每次覆盖), 不会在档案列表里多出一堆副本。
#   fly.sh 的自查照旧: 这份临时配置被别人 (另一台电脑再点一次) 覆盖了 → sha 对不上 → 要 yes。
TEMP_SITE = "临时配置"
TEMP_DIR = os.path.expanduser("~/.flight_monitor/tmp_site")


def _flying():
    s = STORE.snapshot()
    return bool(s["mavros"].get("armed")) or bool(
        s["script_alive"] and (s["telemetry"] or {}).get("air"))


def temp_launcher(args, body):
    if _flying():
        return 409, {"ok": False, "error": "飞机已解锁/在空中 —— 飞行中不生成启动脚本"}
    try:
        knobs = normalize_knobs(body.get("knobs") or {})
        sets = normalize_sets(body.get("sets"))
    except ValueError as e:
        return 400, {"ok": False, "error": str(e)}
    res = dry_run(args, knobs, sets)
    if not res.get("ok"):
        return 422, {"ok": False, "error": "校验不通过, 不生成",
                     "errors": res.get("errors", [])}
    os.makedirs(TEMP_DIR, exist_ok=True)
    path = os.path.join(TEMP_DIR, TEMP_SITE + ".conf")
    base = str(body.get("base") or "").strip()
    header = ("临时配置 —— 网页「只生成启动脚本」写的, 没有存为档案 (下次生成会覆盖)\n"
              "%s%s" % (datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        ("  基于档案 %s" % base) if base else "  基于代码默认值"))
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(fp.format_conf(knobs, header=header, sets=sets))
    os.replace(tmp, path)
    return 200, {"ok": True, "path": path,
                 "launcher": write_launcher(args, TEMP_SITE, conf=path, temp=True,
                                            script=script_for(args, knobs)[1])}


# =============================================================================
# 启动脚本 (~/fly.sh): 网页生成, 在普通 SSH 终端里直接运行
# =============================================================================
# 运行时先自查三件事, 任何一件不对就打印警告 + 档案内容, 要求输入 yes 才起飞:
#   · 生成超过 --launcher-max-age 秒 (默认 60) —— 防止拿上一场 / 早上那套配置飞
#   · 档案在生成之后被改过 (sha256 对不上) —— 现在生效的不是网页上预览过的那套
#   · 飞行脚本在生成之后被改过 (更新过代码、flight.py 软链接换了目标)
# 档案或飞行脚本不存在 → 直接拒绝。最后用 exec 把终端交给 python:
# SSH 断开时飞行脚本要能收到 stdin EOF 才会自动降落, 中间不能隔一层 bash。
# 时间戳用的是飞机自己的时钟 (生成和运行都在飞机上), 不需要和笔记本对时。
# 2026-10-30: 模板正文搬到同目录 fly.sh.tmpl (原样, 占位符 @@名字@@)
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "fly.sh.tmpl"),
          encoding="utf-8", newline="") as _f:
    LAUNCHER_TMPL = _f.read()


def _sha256(path):
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _tilde(p):
    home = os.path.expanduser("~")
    return "~" + p[len(home):] if p == home or p.startswith(home + os.sep) else p


def write_launcher(args, name, conf=None, temp=False, script=None):
    """按档案 name 生成启动脚本。调用方负责先校验档案 (dry-run 通过)。
    conf/temp: 临时配置 (不在档案目录里, 见 temp_launcher)。
    script: 用哪个飞行脚本 (按档案的任务选, 见 script_for); 不给 = 默认任务的脚本。"""
    import shlex
    script = script or args.script
    conf = conf or os.path.join(os.path.expanduser(args.sites_dir), name + ".conf")
    path = os.path.expanduser(args.launcher)
    now = time.time()
    vals = {
        "GEN_STR": datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S"),
        "GEN_TS": str(int(now)),
        "MAX_AGE": str(int(args.launcher_max_age)),
        "SITE": name, "CONF": conf, "SCRIPT_PATH": script,
        "LAUNCHER": _tilde(path),
        "Q_SITE": shlex.quote(name), "Q_CONF": shlex.quote(conf),
        "Q_SITES": shlex.quote(os.path.dirname(conf)),
        "CONF_SHA": _sha256(conf), "SCRIPT_SHA": _sha256(script),
        "Q_DIR": shlex.quote(os.path.dirname(script)),
        "Q_SCRIPT": shlex.quote(os.path.basename(script)),
        "TEMP": "1" if temp else "0",
    }
    text = LAUNCHER_TMPL
    for k, v in vals.items():
        text = text.replace("@@%s@@" % k, v)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    os.chmod(tmp, 0o755)
    os.replace(tmp, path)
    STORE.add_event("INFO", "已生成启动脚本 %s (%s)" % (
        _tilde(path), "临时配置, 未存档案" if temp else "档案 " + name), "monitor")
    return {"path": path, "command": _tilde(path), "gen_ts": now, "site": name, "temp": temp}


def launcher_status(args):
    """读出现有启动脚本的元数据, 并像它运行时一样判断是否会警告。"""
    path = os.path.expanduser(args.launcher)
    st = {"exists": False, "path": path, "command": _tilde(path),
          "max_age": args.launcher_max_age, "now": time.time()}
    if not os.path.exists(path):
        return st
    meta = {}
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                m = re.match(r"^(GEN_TS|SITE|CONF|CONF_SHA|SCRIPT_DIR|SCRIPT|SCRIPT_SHA|TEMP)=(.*)$",
                             line.rstrip("\n"))
                if m:
                    meta[m.group(1)] = m.group(2).strip("'")
    except OSError as e:
        st["error"] = str(e)
        return st
    if "GEN_TS" not in meta:
        st["error"] = "不是网页生成的启动脚本"
        return st
    st.update(exists=True, site=meta.get("SITE"), gen_ts=float(meta["GEN_TS"]),
              temp=meta.get("TEMP") == "1")
    conf = meta.get("CONF", "")
    scr = os.path.join(meta.get("SCRIPT_DIR", ""), meta.get("SCRIPT", ""))
    st["conf_missing"] = not os.path.exists(conf)
    st["conf_changed"] = (not st["conf_missing"]) and _sha256(conf) != meta.get("CONF_SHA")
    st["script_missing"] = not os.path.exists(scr)
    st["script_changed"] = (not st["script_missing"]) and _sha256(scr) != meta.get("SCRIPT_SHA")
    st["script"] = scr
    return st


def regen_launcher(args, body):
    """按已保存的档案重新生成启动脚本 (先重新跑一遍 dry-run 校验)。"""
    name = str(body.get("name", "")).strip()
    if not NAME_RE.match(name) or name.startswith("__"):
        return 400, {"ok": False, "error": "档案名不合法"}
    if _flying():
        return 409, {"ok": False, "error": "飞机已解锁/在空中 —— 飞行中不生成启动脚本"}
    temp = bool(body.get("temp"))
    knobs, sets, path, errs = fp.load_site(name, TEMP_DIR if temp else args.sites_dir)
    if errs:
        return 422, {"ok": False, "error": "档案读不懂", "errors": errs}
    res = dry_run(args, knobs or {}, sets or [])
    if not res.get("ok"):
        return 422, {"ok": False, "error": "校验不通过, 不生成", "errors": res.get("errors", [])}
    return 200, {"ok": True, "launcher": write_launcher(
        args, name, conf=path if temp else None, temp=temp,
        script=script_for(args, knobs or {})[1])}
