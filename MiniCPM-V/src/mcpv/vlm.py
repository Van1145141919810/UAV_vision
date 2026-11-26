# -*- coding: utf-8 -*-
"""MiniCPM-V 客户端（走 Ollama 的 /api/generate）。

为什么不用 `ollama run` 命令行：
    它会"猜"提示词里哪个词是图片路径，猜不到就**静默丢图**、不报错，
    模型只收到文字，于是拿文件名编故事（我们被坑过一次）。
    这里用 images 字段显式传 base64，传没传一目了然。
"""

import base64
import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Tuple

import cv2

from . import config as C


class VLMError(RuntimeError):
    pass


def _encode_jpeg(bgr) -> str:
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    if not ok:
        raise VLMError("图像编码失败")
    return base64.b64encode(buf.tobytes()).decode("ascii")


def ask_crop(bgr, prompt: str = C.VLM_PROMPT,
             timeout: float = None) -> Tuple[str, float]:
    """问一张裁剪图，返回 (回答, 耗时秒)。回答会截断成前 8 个字。"""
    payload = {
        "model": C.VLM_MODEL,
        "prompt": prompt,
        "images": [_encode_jpeg(bgr)],
        "stream": False,
        "options": {"temperature": 0},      # 分类任务用贪心，可复现
        "keep_alive": C.VLM_KEEP_ALIVE,
        "think": False,                      # 关掉思考，快 4 倍
    }
    req = urllib.request.Request(
        C.VLM_HOST + "/api/generate",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    t0 = time.time()
    try:
        resp = json.loads(urllib.request.urlopen(req, timeout=C.VLM_TIMEOUT).read())
    except urllib.error.URLError as e:
        raise VLMError(
            f"连不上 Ollama（{C.VLM_HOST}）：{e}\n"
            f"    先确认 `ollama list` 能跑；首次运行要 `ollama pull {C.VLM_MODEL}`"
        ) from e
    return (resp.get("response") or "").strip(), time.time() - t0


def parse_verdict(answer: str) -> str:
    """把模型回答收敛成 "A" / "B" / "?"。"""
    a = (answer or "").strip()
    if not a:
        return "?"
    head = a[0].upper()
    if head in ("A", "B"):
        return head
    if "标记" in a[:6] or "是" == a[:1]:
        return "A"
    if "不是" in a[:4]:
        return "B"
    return "?"
