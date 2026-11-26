# -*- coding: utf-8 -*-
"""把一次"选定 + 语义 + 几何"打包成一份 JSON，这就是交给 Jev 的交接单。

文件落在 data/results/record_<时间戳>.json，每按一次"发送"生成一个。
同时追加一行到 data/results/session.jsonl，方便事后一次性回放。
"""

import json
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from . import config as C


def make_record(image_path: str, image_size, selection: dict,
                cmd, scene: Optional[dict] = None) -> dict:
    """组装一份记录。

    selection: {"kind": "win"/"dot"/"point", "bbox_px": [...] or "point": [u,v]}
    cmd:       aim.MoveCommand
    scene:     scene.describe_target() 的返回值（没跑就是 None）
    """
    return {
        "schema": "mcpv/record-v1",
        "t_capture": time.time(),
        "time_str": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "image": str(image_path),
        "image_size": list(image_size),
        "selection": selection,
        "vlm": scene or {"window_state": "unknown", "glare": "unknown", "skipped": True},
        "geometry": cmd.to_dict(),
    }


def save_record(rec: dict, out_dir: Optional[Path] = None) -> Path:
    out_dir = Path(out_dir or C.RESULTS_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = out_dir / f"record_{stamp}.json"
    path.write_text(json.dumps(rec, ensure_ascii=False, indent=2), encoding="utf-8")

    with open(out_dir / "session.jsonl", "a", encoding="utf-8") as fp:
        fp.write(json.dumps(rec, ensure_ascii=False) + "\n")
    return path
