# -*- coding: utf-8 -*-
"""
flight_profile.py —— 现场档案 (兼容入口, 2026-10-30)
===============================================================================
代码在 vb09/profile/core.py (解析 / 格式化 / --set / setup / finalize / JSON) 和 vb09/profile/gate.py
(穿环旋钮 / 校验 / 派生 / 打印), 原样搬过去的; 设计原则和档案格式的完整说明在 core.py 文件头。
这个文件把两边的名字全部导出, 所以 `import flight_profile as fp; fp.setup(...)` / `fp.KNOBS` 照用;
单独运行也照旧:
    python3 flight_profile.py               列出现有档案
    python3 flight_profile.py 二楼          只做旋钮级检查
    python3 flight_profile.py --schema      输出旋钮表 JSON (网页表单用)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vb09.profile import core as _core  # noqa: E402
from vb09.profile import gate as _gate  # noqa: E402

globals().update({k: v for k, v in vars(_core).items() if not k.startswith("__") and k != "_g"})
globals().update({k: v for k, v in vars(_gate).items() if not k.startswith("__") and k != "_c"})


# =============================================================================
# 直接运行
# =============================================================================
if __name__ == "__main__":
    args = sys.argv[1:]
    d = DEFAULT_SITES_DIR
    if "--sites-dir" in args:
        i = args.index("--sites-dir")
        d = args[i + 1]
        del args[i:i + 2]
    if "--schema" in args:
        print(json.dumps(schema_json(), ensure_ascii=False, indent=1))
        sys.exit(0)
    if not args:
        print("用法: python3 flight_profile.py <站点名> [--sites-dir DIR]")
        print("      python3 flight_profile.py --schema")
        print("现有档案:")
        for n in list_sites(d):
            print("   " + n)
        sys.exit(0)
    name = args[0]
    knobs, sets, path, errs = load_site(name, d)
    if errs:
        _die({"src": path, "errors": errs, "warnings": []})
    e, w = validate_knobs(knobs)
    _task = "gate"
    e += check_knob_task(knobs, _task, "任务「穿环」")
    ov, rep, w2 = build_overrides(knobs, dict(_STANDALONE_DEFAULTS), _task)
    if sets:
        rep.append("档案 set. 覆盖(单独运行时不检查参数是否存在): " + ", ".join(sets))
    for x in e:
        print("  ✗ " + x)
    for x in w + w2:
        print("  ⚠️ " + x)
    if e:
        sys.exit(1)
    print("档案 %s 旋钮级检查通过" % path)
    for l in rep:
        print("  · " + l)
    print("  (具体参数覆盖 %d 个)" % len(ov))
    print("  ★ 这只是旋钮级检查。完整检查(含代码默认值和派生量)请用:")
    print("      python3 %s --site %s --dry-run"
          % ("stageNN_<任务>.py", name))
