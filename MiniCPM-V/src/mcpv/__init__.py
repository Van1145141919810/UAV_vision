# -*- coding: utf-8 -*-
"""mcpv —— 无人机画面里的"红色圆形标记"检测（MiniCPM-V + 传统 CV 混合）。

链路：
    CV 找候选  →  白底环形带打分过滤  →  裁剪放大  →  VLM 闭集判定  →  JSON + 标注图

用法见项目根目录 README.md。
"""

__version__ = "0.1.0"
