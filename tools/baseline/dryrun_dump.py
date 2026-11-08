# -*- coding: utf-8 -*-
"""
dryrun_dump.py —— 档案 → `<入口> --dry-run [--json]` 输出等价验证
  每个 sites/*.conf + 几组命令行 (无档案 / --set / 非法参数) 各跑 JSON 和文本两遍, 存 rc + 输出。
判定: 逐字节相同。例外 (白名单):
  · 路径: 仓库根目录换成 <ROOT>, 临时 HOME 换成 <HOME>
  · 戳点删除那一步: JSON 里键名含 POKE / poke 的字段、文本里含"戳"/"POKE"的行, 两边都去掉再比;
    EXPECT_CHANGED 里的用例 (戳点档案 / 跟踪档案) 只记录新输出, 不比 (行为本来就该变: 改成报错)
"""
import json
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402

SITES = os.path.join(_compat.HERE, "sites")
# 有意改变行为的用例 → {用例名: (原因, 期望 rc, 输出里必须出现的文字)}。
#   输出不和 golden 逐字比 (golden 里留旧输出供对照), 只查 rc 和关键文字。
EXPECT_CHANGED = {
    # 第 1a 步 (删戳点)
    "poke": ("gate_mode = 戳点 → 报错拒绝 (不能悄悄当冲刺飞)", 1, "戳点模式已删除"),
    "dash_mode": ("gate_mode = 冲刺 → 警告并忽略 (已另行核对: 去掉这条警告后和不写 gate_mode 的档案逐字相同)",
                  0, "gate_mode (穿环模式) 已删除, 忽略"),
    "typo": ("未知键报错里的'允许的键'列表少了戳点 / 跟踪旋钮", 1, "未知键 'z_hig'"),
    # 第 1b 步 (删停靠跟踪)
    "track": ("跟踪档案照样拒绝, 原因从'任务不符'变成'未知键' (跟踪旋钮已删除)", 1, "未知键 'standoff'"),
    # 第 4.5 步 (mode 旋钮)
    "shadow": ("旧写法 set.GATE_SHADOW = True: 多一条'等同于 mode = 影子'的提示, 其余不变", 0, "旧写法, 现在等同于 mode = 影子"),
    "set_shadow_cli": ("命令行 --set GATE_SHADOW=True: 同上", 0, "旧写法, 现在等同于 mode = 影子"),
    "mode_dash": ("新档案: mode = 冲刺", 0, "任务模式: 冲刺"),
    "mode_conflict": ("mode = 冲刺 + set.GATE_SHADOW = True → 警告, 按影子飞", 0, "两个说法矛盾, 按 影子 飞"),
    # 第 7 步 (停靠, 新功能): 新用例, golden 里没有
    "mode_dock_conflict": ("mode = 停靠 + set.GATE_SHADOW = True → 警告, 按影子飞", 0, "mode = 停靠, 但 set.GATE_SHADOW = True"),
    "dock_close_cli": ("--set GATE_DOCK_D_STOP=1.0 → 报错 (太近)", 1, "停靠距离 1.00 m 太近"),
    "dock_hold_zero_cli": ("--set GATE_DOCK_HOLD_S=0 → 报错 (必须是正数)", 1, "GATE_DOCK_HOLD_S=0"),
    # 2026-11-06 模式分区: 停靠不做冲刺检查、不用的旋钮越界只提示; 影子 / 停靠的 gate 速度包络按摆位速度
    "dock_small": ("小房间停靠: 环后空间 1 m / dash_v 越界都不拒绝", 0, "其中越界的没有写进参数"),
    "dash_reddot": ("红点 + 冲刺: 实验性组合, 警告", 0, "目标 红点 + 模式 冲刺 是实验性的"),
    "mode_dock": ("停靠: 不再做冲刺检查; 冲刺旋钮列为'当前模式不用'; gate 包络 1.2 m/s (以前按 DASH_V)", 0,
                  "停靠模式 (mode = 停靠): 识别 → 飞到目标前 4.00 m"),
    "mode_shadow": ("影子: gate 速度包络按摆位速度 (以前按 DASH_V)", 0, "任务模式: 影子"),
}
# 第 7 步新增的参数 (停靠): 所有用例的 params 里都多了它们 —— 比较前两边去掉
# 新参数: 停靠 (第 7 步), 上下扫描 (2026-11-05), 红点抵近 / 选点 / 换视角 / 重锁 (2026-11-08, PLAN_dot_v2)
_NEW_KEY = re.compile(r"^(GATE_(DOCK|VSCAN|APPR|REFINE|SELECT|RELOCK)_|DOT_(D_MAX|SD_|SEL_))")


def cases():
    out = []
    for f in sorted(os.listdir(SITES)):
        if f.endswith(".conf"):
            out.append((f[:-5], ["--sites-dir", SITES, "--site", f[:-5]]))
    out += [
        ("nosite", []),
        ("set_cli", ["--sites-dir", SITES, "--site", "sim", "--set", "DASH_V=2.0", "--set", "GATE_HOLD_S=2"]),
        ("set_bad", ["--sites-dir", SITES, "--site", "sim", "--set", "NOPE=1"]),
        ("set_shadow_cli", ["--sites-dir", SITES, "--site", "sim", "--set", "GATE_SHADOW=True"]),
        ("bad_flag", ["--sites-dir", SITES, "--site", "sim", "--dryrun"]),
        ("missing_site", ["--sites-dir", SITES, "--site", "nope"]),
        ("dock_close_cli", ["--sites-dir", SITES, "--site", "mode_dock", "--set", "GATE_DOCK_D_STOP=1.0"]),
        ("dock_hold_zero_cli", ["--sites-dir", SITES, "--site", "mode_dock", "--set", "GATE_DOCK_HOLD_S=0"]),
    ]
    return out


def run_one(entry, args, js, home):
    env = dict(os.environ, HOME=home, PYTHONHASHSEED="0")
    for k in ("TMUX", "STY", "TERM_PROGRAM", "VSCODE_IPC_HOOK_CLI"):
        env.pop(k, None)
    cmd = [sys.executable, entry] + args + ["--dry-run"] + (["--json"] if js else [])
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                       cwd=_compat.ROOT, stdin=subprocess.DEVNULL, timeout=120)
    txt = p.stdout.decode("utf-8", "replace").replace(_compat.ROOT, "<ROOT>").replace(home, "<HOME>")
    txt = txt.replace(os.path.basename(entry), "<ENTRY>")
    return {"rc": p.returncode, "out": txt}


def run():
    entry = _compat.entry()
    home = tempfile.mkdtemp()
    res = {}
    for name, args in cases():
        for js in (True, False):
            r = run_one(entry, args, js, home)
            if js:
                try:
                    r["json"] = json.loads(r["out"][r["out"].index("{"):])
                    r.pop("out")
                except ValueError:
                    pass
            res["%s%s" % (name, ".json" if js else ".txt")] = r
    return res


_POKE_KEY = re.compile(r"poke|POKE")


def _drop_json(x):
    if isinstance(x, dict):
        return {k: _drop_json(v) for k, v in x.items() if not _POKE_KEY.search(k) and not _NEW_KEY.search(k)}
    if isinstance(x, list):
        return [_drop_json(v) for v in x
                if not (isinstance(v, str) and ("戳" in v or "POKE" in v))]
    return x


def _drop_mode_info(d):
    """2026-11-06 模式分区: derived 多了 mode / dock 两个键 (网页用), 比较前去掉。"""
    if isinstance(d, dict):
        js = d.get("json")
        if isinstance(js, dict) and isinstance(js.get("derived"), dict):
            js["derived"].pop("mode", None)
            js["derived"].pop("dock", None)
    return d


_MODE_LINE = re.compile(r"^(\s*)模式          \S+ —— .*$", re.M)


def _drop_txt(s):
    s = s.replace("穿环 (stage12)", "穿环 (模式见下面 开关 / 档案 mode)")      # 4.6 改名后的措辞
    # 2026-11-06: 核对表第一行从"任务 穿环"改成"模式 <名> —— <说明>"
    s = _MODE_LINE.sub(lambda m: m.group(1) + "任务          穿环 (模式见下面 开关 / 档案 mode)", s)
    return "\n".join(ln for ln in s.splitlines() if "戳" not in ln and "POKE" not in ln and "GATE_DOCK_" not in ln and "GATE_VSCAN_" not in ln)


CHANGE_FAIL = []


def drop(g, n):
    for k, (_why, rc, must) in EXPECT_CHANGED.items():
        for suf in (".json", ".txt"):
            r = n.get(k + suf) or {}
            text = json.dumps(r, ensure_ascii=False)
            if r.get("rc") != rc or must not in text:
                CHANGE_FAIL.append("%s%s: rc=%s (要 %s), 含 %r: %s" % (k, suf, r.get("rc"), rc, must, must in text))
    for k in list(EXPECT_CHANGED):
        for suf in (".json", ".txt"):
            g.pop(k + suf, None)
            n.pop(k + suf, None)
    for d in (g, n):
        for v in d.values():
            _drop_mode_info(v)
    g, n = _drop_json(g), _drop_json(n)
    for d in (g, n):
        for k, v in d.items():
            if isinstance(v, dict) and isinstance(v.get("out"), str):
                v["out"] = _drop_txt(v["out"])
    return g, n


def main():
    write = _util.main_args()
    r = run()
    ok = _util.report("dryrun", *_util.save_or_compare("dryrun.json", r, write,
                                                       drop=None if write else drop))
    if not write and EXPECT_CHANGED:
        for k, (why, _rc, _m) in EXPECT_CHANGED.items():
            j = r.get(k + ".json", {})
            print("   (expected change) %-10s rc=%s  %s" % (k, j.get("rc"), why))
        for x in CHANGE_FAIL:
            print("   ✗ expected-change check failed: " + x)
        ok = ok and not CHANGE_FAIL
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
