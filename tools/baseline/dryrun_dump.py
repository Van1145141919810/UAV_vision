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
# 有意改变行为的用例 → {用例名: 原因}。只在 golden 里留旧输出供对照, 不参与比较。
EXPECT_CHANGED = {}


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
        return {k: _drop_json(v) for k, v in x.items() if not _POKE_KEY.search(k)}
    if isinstance(x, list):
        return [_drop_json(v) for v in x
                if not (isinstance(v, str) and ("戳" in v or "POKE" in v))]
    return x


def _drop_txt(s):
    return "\n".join(ln for ln in s.splitlines() if "戳" not in ln and "POKE" not in ln)


def drop(g, n):
    for k in list(EXPECT_CHANGED):
        for suf in (".json", ".txt"):
            g.pop(k + suf, None)
            n.pop(k + suf, None)
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
        for k, why in EXPECT_CHANGED.items():
            j = r.get(k + ".json", {})
            print("   (expected change) %-12s rc=%s  %s" % (k, j.get("rc"), why))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
