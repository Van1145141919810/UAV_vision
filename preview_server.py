# -*- coding: utf-8 -*-
"""
preview_server.py —— 兼容壳 (2026-10-30)
===============================================================================
代码搬到 vb09/common/preview_server.py (原样)。这个文件留着是因为 gcs_aruco_det2.py (本轮不改)
从自己所在目录 `import preview_server`。用法不变:
    import preview_server; preview_server.start(); preview_server.publish(img, meta=..., raw=...)
    python3 preview_server.py --demo
注意: 两种 import 拿到的是**同一个**模块对象 (sys.modules 里换成了 vb09.common.preview_server),
所以 start() / publish() 共用一份状态。
"""
import os
import runpy
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

if __name__ == "__main__":
    runpy.run_module("vb09.common.preview_server", run_name="__main__")
else:
    from vb09.common import preview_server as _impl
    sys.modules[__name__] = _impl
