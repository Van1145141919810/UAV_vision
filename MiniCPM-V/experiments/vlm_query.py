# -*- coding: utf-8 -*-
"""
vlm_query.py —— 问本地 MiniCPM-V 一张图，并把耗时拆成三段。

为什么不用 `ollama run`：
    CLI 走"在提示词里猜哪个是图片路径"，猜不到会把图**悄悄丢掉**、不报错，
    模型只收到文字，于是拿文件名编故事。API 的 images 字段是显式传的。

为什么要拆三段：
    一次请求的墙钟时间 = load（读进显存）+ prefill（看图）+ decode（写字）。
    优化之前先知道钱花在哪一段，否则调半天调错地方。

用法：
    python vlm_query.py 图片
    python vlm_query.py 图片 "问题"
    python vlm_query.py 图片 "问题" --max-width 700     # 先缩图再送（快，但丢细节）
    python vlm_query.py 图片 "问题" --repeat 3          # 同一问题采样 3 次，看稳不稳
    python vlm_query.py 图片 "问题" --think             # 开思考模式（慢，一般不用）
"""

import sys
import os
import io
import json
import time
import base64
import tempfile
import urllib.request
import urllib.error

HOST = "http://127.0.0.1:11434"
MODEL = "openbmb/minicpm-v4.7:1b"


def fix_console():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
        except Exception:
            pass
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass


def maybe_resize(path, max_width):
    """超过 max_width 就先缩一下。返回实际要读的路径。"""
    if not max_width:
        return path, None
    try:
        from PIL import Image
    except ImportError:
        print("[!] 没装 Pillow，跳过缩图（pip install pillow）")
        return path, None
    im = Image.open(path)
    if im.width <= max_width:
        return path, im.size
    h = int(im.height * max_width / im.width)
    out = os.path.join(tempfile.gettempdir(), "mcpv_resized.jpg")
    im.convert("RGB").resize((max_width, h), Image.LANCZOS).save(out, quality=92)
    return out, (max_width, h)


def ask(img_path, prompt, think=False, keep_alive="30m", timeout=600):
    raw = open(img_path, "rb").read()
    payload = {
        "model": MODEL,
        "prompt": prompt,
        "images": [base64.b64encode(raw).decode("ascii")],
        "stream": False,
        "options": {"temperature": 0},
        "keep_alive": keep_alive,
    }
    if not think:
        payload["think"] = False

    req = urllib.request.Request(
        HOST + "/api/generate",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    t0 = time.time()
    resp = json.loads(urllib.request.urlopen(req, timeout=timeout).read())
    return resp, time.time() - t0


def main():
    fix_console()
    argv = sys.argv[1:]
    flags = {a for a in argv if a.startswith("--")}
    args = [a for a in argv if not a.startswith("--")]

    def opt(name, default=None):
        if name in argv:
            i = argv.index(name)
            if i + 1 < len(argv):
                return argv[i + 1]
        return default

    if not args:
        print(__doc__)
        return 1

    img = args[0]
    prompt = args[1] if len(args) > 1 else "用一句话描述这张图。"
    think = "--think" in flags
    max_width = opt("--max-width")
    max_width = int(max_width) if max_width else None
    repeat = int(opt("--repeat", "1"))

    if not os.path.isfile(img):
        print(f"[x] 找不到图片: {img}")
        return 1

    send_path, new_size = maybe_resize(img, max_width)

    for k in range(repeat):
        try:
            resp, dt = ask(send_path, prompt, think=think)
        except urllib.error.URLError as e:
            print(f"[x] 连不上 Ollama ({HOST})：{e}")
            print("    先确认 `ollama list` 能跑")
            return 1

        ns = lambda x: x / 1e9
        load = ns(resp.get("load_duration", 0))
        pre = ns(resp.get("prompt_eval_duration", 0))
        dec = ns(resp.get("eval_duration", 0))
        p_tok = resp.get("prompt_eval_count", 0)
        o_tok = resp.get("eval_count", 0)
        ans = resp.get("response", "").strip()

        print("=" * 62)
        print(f"图片 : {os.path.basename(img)}"
              + (f"  → 缩到 {new_size[0]}x{new_size[1]}" if new_size else ""))
        print(f"问题 : {prompt}")
        if repeat > 1:
            print(f"第 {k+1}/{repeat} 次")
        print(f"耗时 : {dt:5.2f}s  =  load {load:.2f}  +  prefill {pre:5.2f}  +  decode {dec:.2f}")
        print(f"token: 图 {p_tok} tok  →  答 {o_tok} tok"
              + (f"   (decode {o_tok/dec:.1f} tok/s)" if dec > 0 else ""))
        print("-" * 62)
        print(ans if ans else "(空回答)")
    print("=" * 62)
    return 0


if __name__ == "__main__":
    sys.exit(main())
