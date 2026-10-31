# -*- coding: utf-8 -*-
"""
tools/wiring_check.py —— 感知插件接线自检 (2026-11-01)
    python3 tools/wiring_check.py        全过打印 ALL PASS, 否则退出码 1
  · registry 每张表的每个 kind 都能 load, 接口方法齐
  · 未知 kind → ValueError, 信息里列出可选项
  · 飞行脚本 / 机上融合要用的 (JUDGES / LOCATORS / CODECS / CONFIGS) 不会把 cv2 拉进来 (子进程里查)
  · wire.frame_to_msg / frame_from_msg 分发 = 环自己的实现
  · peer_mismatch: 目标类型不同 → 报; 旧格式 rig (没有 kind) 当 ring
  · 档案校验: TARGET_KIND 不认识 → 报错
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from vb09.perception import registry  # noqa: E402

BAD = []


def check(name, ok, msg=""):
    print("%-4s %-40s %s" % ("ok" if ok else "FAIL", name, msg))
    if not ok:
        BAD.append(name)


IFACE = {
    "CODECS": ("MSG_VERSION", "frame_to_msg", "frame_from_msg", "LIB_FILES"),
    "CONFIGS": ("load",),
    "DETECTORS": ("process", "reset", "draw", "extra"),
    "LOCATORS": ("same_rig", "update", "extra"),
    "JUDGES": ("judge",),
}
for tname, attrs in IFACE.items():
    table = getattr(registry, tname)
    for kind in table:
        try:
            obj = registry.load(table, kind)
            miss = [a for a in attrs if not hasattr(obj, a)]
            check("%s[%s]" % (tname, kind), not miss, "缺 %s" % miss if miss else "")
        except Exception as e:                       # noqa: BLE001
            check("%s[%s]" % (tname, kind), False, repr(e))

try:
    registry.load(registry.JUDGES, "nope")
    check("unknown kind → ValueError", False, "没抛")
except ValueError as e:
    check("unknown kind → ValueError", "ring" in str(e), str(e))

code = ("import sys; sys.path.insert(0, %r); sys.path.insert(0, %r)\n"
        "from vb09.perception import registry as r\n"
        "for t in (r.JUDGES, r.LOCATORS, r.CODECS, r.CONFIGS):\n"
        "    [r.load(t, k) for k in t]\n"
        "from vb09.common import wire\n"
        "print('cv2' in sys.modules)\n") % (ROOT, os.path.join(ROOT, "sim", "stubs"))
out = subprocess.run([sys.executable, "-c", code], stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout.decode()
check("飞行 / 机上融合不拉 cv2", out.strip().endswith("False"), out.strip().splitlines()[-1] if out.strip() else "")

from vb09.common import wire  # noqa: E402
from vb09.perception.ring import wire as ring_wire  # noqa: E402
check("wire 分发 = ring.wire", wire.codec("ring") is ring_wire)
cfg = registry.load(registry.CONFIGS, "ring")()
good = {"lib": wire.lib_version("ring"), "cfg": wire.cfg_digest(cfg)}
gm = {"v": ring_wire.MSG_VERSION}
check("peer_mismatch: 旧 rig (没有 kind) = ring", wire.peer_mismatch(gm, dict(good), cfg) == [])
check("peer_mismatch: kind 一致", wire.peer_mismatch(gm, dict(good, kind="ring"), cfg, "ring") == [])
bad = wire.peer_mismatch(gm, dict(good, kind="reddot"), cfg, "ring")
check("peer_mismatch: kind 不同 → 报", bool(bad) and "目标类型" in bad[0], str(bad))
check("lib_files 含公共 + 环", set(wire.COMMON_LIB_FILES) <= set(wire.lib_files("ring"))
      and "vb09/perception/ring/plugin.py" in wire.lib_files("ring"))
missing = [f for f in wire.lib_files("ring") if not os.path.exists(os.path.join(ROOT, f))]
check("lib_files 都存在", not missing, str(missing))

from vb09.profile import gate as pg  # noqa: E402
import vb09.flight.config as C  # noqa: E402
C.derive()
g = {k: v for k, v in vars(C).items() if k.isupper()}
E, _W = pg._check_gate(dict(g, TARGET_KIND="nope"))
check("档案校验: TARGET_KIND 不认识 → 报错", any("TARGET_KIND" in e for e in E), "; ".join(e for e in E if "TARGET" in e))
E0, _ = pg._check_gate(dict(g))
check("档案校验: TARGET_KIND=ring 不报", not any("TARGET_KIND" in e for e in E0))

print("=" * 60)
print("ALL PASS" if not BAD else "%d FAIL: %s" % (len(BAD), BAD))
sys.exit(1 if BAD else 0)
