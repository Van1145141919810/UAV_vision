# -*- coding: utf-8 -*-
"""输出结构定义。

这份 JSON 就是交给下游（Jev / 文豪的飞行脚本）的契约。
字段含义写在这里，改字段记得同步告诉对方。
"""

from dataclasses import dataclass, field, asdict
from typing import List, Optional


@dataclass
class Candidate:
    """一个红色候选。"""

    cx: int                     # 中心 x（原图像素）
    cy: int                     # 中心 y
    diameter_px: float          # 等效直径（像素）
    bbox_px: List[int]          # [x0, y0, x1, y1]
    circularity: float          # 圆度 4πA/P²
    ring_s: float               # 环带平均饱和度
    ring_v: float               # 环带平均亮度
    score: float                # 白底打分 V - 1.5S（浅色底特征）
    cdiff: float = 0.0          # 局部色度对比 = 点内色度 - 环带色度（主排序键）
    passed_cv: bool = True      # 是否通过候选门槛（圆度 + 尺寸）
    white_ok: bool = False      # 环形带是否像"浅色底"（特征，不参与筛除）
    vlm_verdict: Optional[str] = None   # "A" / "B" / None（没问）
    vlm_text: Optional[str] = None      # 模型原话（证据，不参与判断）

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class DetectionResult:
    """一次检测的完整结果。"""

    image: str                          # 文件名
    image_path: str                     # 绝对路径
    image_size: List[int]               # [宽, 高]
    has_target: bool                    # 结论：画面里有没有目标
    stage: str                          # no_candidate / cv_only / vlm
    n_candidates: int                   # CV 找到几个候选
    n_passed_cv: int                    # 通过白底阈值几个
    elapsed_s: float                    # 总耗时（秒）
    vlm_calls: int                      # 实际调用 VLM 次数
    candidates: List[Candidate] = field(default_factory=list)

    @property
    def best(self) -> Optional[Candidate]:
        """判定为目标的那一个（没有就是 None）。"""
        for c in self.candidates:
            if c.vlm_verdict == "A":
                return c
        return None

    def to_dict(self) -> dict:
        d = asdict(self)
        b = self.best
        d["best"] = None if b is None else {
            "bbox_px": b.bbox_px,
            "mode": b.vlm_verdict or ("A" if b.passed_cv else "B"),
            "score": round(b.score, 1),
        }
        return d
