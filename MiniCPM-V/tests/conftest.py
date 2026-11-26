# -*- coding: utf-8 -*-
"""让 pytest 能找到 src/mcpv（不装包也能跑）。"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
