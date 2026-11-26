# -*- coding: utf-8 -*-
r"""MiniCPM-V 的"语义层"：判断这块区域是不是窗户、窗开着还是关着、反光强不强。

为什么要有这一层：
    CV 能给 bbox、能给像素数，但给不出"窗是开着的""玻璃反光很强"。
    这两类信息只有 VLM 能给，而它们恰好决定"这一步该不该继续靠近"。

踩过的坑（别改回去）：
    1. 第一版问法是「窗=开着/关着/看不清」，模型把选项原样抄回来
       （回答就是 "窗=关着/看不清"），根本没法解析。改成单字母 A/B/C/D 才稳。
    2. 不先问"这里是不是窗户"的话，对着白纸上的红点它会硬答"开着"。
       所以先过一道 Q_KIND 门槛。

代价：裁剪图 384×384（约 250 token），热态一次 0.8~1.3 s；
      窗户问 3 次（约 2.5 s），非窗户问 2 次（约 1.8 s）。
"""

import numpy as np

from .vlm import ask_crop
from .prompts import (P_IS_WINDOW, P_WINDOW_STATE, P_GLARE, P_DESCRIBE)

# ★ 2026-11-05 修正：这四个问题原来在这里另写了一套措辞，和评测脚本
#   eval/run_eval.py 里的**不是同一句话** ——
#     线上"反光"问的是 C/D，评测问的是 A/B；
#     线上"开合"只给 2 个选项，评测给 3 个（多一个"看不清"）。
#   后果：**评测出来的数字并不描述线上实际问法**，两边对不上。
#   现在统一到评测验证过的那一套（见 prompts.py，版本 v1_20261105）。
#   以后改提示词只改 prompts.py 一处。
Q_KIND = P_IS_WINDOW
Q_OPEN = P_WINDOW_STATE
Q_GLARE = P_GLARE
Q_NOTE = P_DESCRIBE
# 目标定义（用户 2026-11-04 明确）：红圆必须是【贴在窗户上的】，贴在墙/地/招牌上都算错。
# 但当前测试集是贴在白纸上的，所以用 config.REQUIRE_ON_WINDOW 控制是否强制这一条。
Q_ON_WINDOW = ("画面正中心的红色物体，是不是贴在【窗户玻璃】或【贴在窗上的纸】上？"
               "只回答一个字母：A（是，在窗户上）或 B（不是，在墙/砖/地面/招牌/设备上）。")


def _letter(text: str, allowed: str):
    t = (text or "").strip().upper()
    for ch in t[:6]:
        if ch in allowed:
            return ch
    return None


def describe_target(crop_bgr: np.ndarray, want_note: bool = False) -> dict:
    """返回 {is_window, window_state, glare, note, raw, elapsed_s}。"""
    a0, dt0 = ask_crop(crop_bgr, Q_KIND)
    is_win = _letter(a0, "AB") == "A"

    if is_win:
        a1, dt1 = ask_crop(crop_bgr, Q_OPEN)
        state = {"A": "开着", "B": "关着"}.get(_letter(a1, "AB"), "unknown")
    else:
        a1, dt1 = "skip", 0.0
        state = "n/a"

    a2, dt2 = ask_crop(crop_bgr, Q_GLARE)
    glare = {"C": "有", "D": "没有"}.get(_letter(a2, "CD"), "unknown")

    note = ""
    if want_note:
        note, _ = ask_crop(crop_bgr, Q_NOTE)
        note = note.strip().splitlines()[0][:40] if note else ""

    return {
        # 作用域：这一栏只描述"你选的那一小块"，不是整张图。
        # 不写清楚，Jev 会以为"整张立面所有窗都关着"。
        "scope": "selected_region",
        "is_window": is_win,
        "window_state": state,
        "glare": glare,
        "note": note,
        "raw": f"窗户={a0[:8]} / 开合={a1[:10]} / 反光={a2[:10]}",
        "elapsed_s": round(dt0 + dt1 + dt2, 1),
    }


def describe_marker(crop_bgr: np.ndarray) -> dict:
    """红点专用：这块区域中心的红物体，是标记还是别的红东西。

    和 describe_target 分开，因为问题完全不同：
    窗户问“开没开”，红点问“是不是标记”。
    """
    from . import config as C
    from .vlm import parse_verdict

    ans, dt = ask_crop(crop_bgr, C.VLM_PROMPT)
    marker = parse_verdict(ans)

    # 只有"确定是标记"才追问贴在哪；否则这个问题没有主语，模型会瞎答
    # （实测：对着一扇"没有红点"的窗玻璃问，它答"不在窗户上"）
    on_win, dt2, a2 = "unknown", 0.0, ""
    if marker == "A":
        a2, dt2 = ask_crop(crop_bgr, Q_ON_WINDOW)
        on_win = {"A": "是", "B": "否"}.get(_letter(a2, "AB"), "unknown")

    return {
        "marker": marker,                  # A=像标记 / B=其他红东西 / ?=没看懂
        "on_window": on_win,               # 是否贴在窗户上（仅 marker=A 时才有值）
        "raw": f"{ans[:40]}" + (f" / 贴窗={a2[:8]}" if a2 else ""),
        "elapsed_s": round(dt + dt2, 1),
    }
