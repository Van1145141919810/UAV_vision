# -*- coding: utf-8 -*-
"""winseg_worker.py —— 常驻「窗户理解」工作进程（跑在 venv_det 里）

为什么要单独一个进程
──────────────────────────────────────────────────────────────
SegFormer / Grounding DINO 需要 transformers + torch，而主环境 mcpv 里没有
（mcpv 是检测/飞行用的，不能随便加包）。所以把这两个模型关在这个进程里，
主程序通过 **stdin/stdout 的 JSON 行** 跟它说话 —— 不占端口、不动 mcpv。

模型只加载一次（加载 GDINO 要几秒），之后每帧只花推理时间。

协议（一行一个 JSON，全部 ASCII 转义，避免 GBK 控制台乱码）
  请求  {"id":1,"cmd":"detect","img":"<b64 jpeg>","seg":true,"gdino":true,"short":400}
        {"id":2,"cmd":"ping"}
        {"id":3,"cmd":"quit"}
  响应  {"id":1,"ok":true,"w":1920,"h":1080,
         "seg":{"blobs":[[x,y,w,h,area],...],"ms":352},
         "gdino":{"boxes":[[x1,y1,x2,y2,score,"label"],...],"ms":2657},
         "ms":3009}
        {"id":2,"ok":true,"models":{"seg":true,"gdino":true}}

用法（主程序不需要手动起，window_finder 会自动 spawn）
    venv_det\\Scripts\\python.exe scripts\\winseg_worker.py
"""
from __future__ import annotations

import base64
import json
import os
import sys
import time

# ── 把真正的 stdout 留出来给 JSON，其余全部赶到 stderr ────────────────
_REAL = os.fdopen(os.dup(1), "w", encoding="utf-8", newline="\n", buffering=1)
os.dup2(2, 1)
os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")
os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")
os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")

import cv2                      # noqa: E402
import numpy as np              # noqa: E402

SEG_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
GD_ID = "IDEA-Research/grounding-dino-tiny"
WIN_LABEL = 3                   # CMP facade: 0 unknown,1 bg,2 facade,3 window,4 door

_state = {"seg": None, "seg_proc": None, "gd": None, "gd_proc": None}


def _log(*a):
    print("[worker]", *a, file=sys.stderr, flush=True)


def _load_seg():
    if _state["seg"] is not None:
        return True
    from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
    t = time.time()
    _state["seg_proc"] = SegformerImageProcessor.from_pretrained(SEG_ID)
    _state["seg"] = SegformerForSemanticSegmentation.from_pretrained(SEG_ID).eval()
    _log("SegFormer 加载 %.1fs" % (time.time() - t))
    return True


def _load_gd():
    if _state["gd"] is not None:
        return True
    from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
    t = time.time()
    _state["gd_proc"] = AutoProcessor.from_pretrained(GD_ID)
    _state["gd"] = AutoModelForZeroShotObjectDetection.from_pretrained(GD_ID).eval()
    _log("GroundingDINO 加载 %.1fs" % (time.time() - t))
    return True


def run_seg(bgr, min_area=1200, min_side=25):
    """返回 window 类的连通块 [[x,y,w,h,area], ...]（按面积降序）+ mask(PNG b64)。"""
    import torch
    from PIL import Image
    _load_seg()
    proc, model = _state["seg_proc"], _state["seg"]
    h, w = bgr.shape[:2]
    inp = proc(images=Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)), return_tensors="pt")
    t0 = time.time()
    with torch.no_grad():
        out = model(**inp)
    up = torch.nn.functional.interpolate(out.logits, size=(h, w), mode="bilinear", align_corners=False)
    pred = up.argmax(1)[0].numpy().astype(np.uint8)
    ms = (time.time() - t0) * 1000
    m = (pred == WIN_LABEL).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, _, st, _ = cv2.connectedComponentsWithStats(m, 8)
    blobs = []
    for i in range(1, n):
        a, x, y, bw, bh = int(st[i, 4]), int(st[i, 0]), int(st[i, 1]), int(st[i, 2]), int(st[i, 3])
        if a >= min_area and bw >= min_side and bh >= min_side:
            blobs.append([x, y, bw, bh, a])
    blobs.sort(key=lambda b: -b[4])
    ok, buf = cv2.imencode(".png", (m * 255))
    mask = base64.b64encode(buf.tobytes()).decode() if ok else None
    return {"blobs": blobs[:40], "ms": round(ms, 1), "mask": mask, "n_all": n - 1}


def run_gd(bgr, prompts, short=400, box_thr=0.22, text_thr=0.22):
    """返回 boxes [[x1,y1,x2,y2,score,label], ...]（按分数降序）。"""
    import torch
    from PIL import Image
    _load_gd()
    proc, model = _state["gd_proc"], _state["gd"]
    try:
        proc.image_processor.size = {"shortest_edge": int(short),
                                     "longest_edge": int(round(short * 1.6667))}
    except Exception as e:
        _log("设置输入尺寸失败(忽略):", e)
    h, w = bgr.shape[:2]
    inp = proc(images=Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)),
               text=prompts, return_tensors="pt")
    t0 = time.time()
    with torch.no_grad():
        out = model(**inp)
    ms = (time.time() - t0) * 1000
    res = proc.post_process_grounded_object_detection(
        out, inp["input_ids"], threshold=box_thr, text_threshold=text_thr,
        target_sizes=[(h, w)])
    r = res[0] if res else {}
    labs = r.get("text_labels") or r.get("labels") or []
    boxes = []
    for i in range(len(r.get("boxes", []))):
        b = [round(float(v), 1) for v in r["boxes"][i].tolist()]
        boxes.append(b + [round(float(r["scores"][i]), 3),
                          str(labs[i]) if i < len(labs) else "?"])
    boxes.sort(key=lambda b: -b[4])
    return {"boxes": boxes[:40], "ms": round(ms, 1)}


def handle(req):
    cmd = req.get("cmd")
    if cmd == "ping":
        return {"ok": True, "models": {"seg": _state["seg"] is not None,
                                       "gd": _state["gd"] is not None}}
    if cmd == "quit":
        return {"ok": True, "bye": True}
    if cmd != "detect":
        return {"ok": False, "err": "unknown cmd %r" % cmd}
    raw = base64.b64decode(req["img"])
    bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        return {"ok": False, "err": "decode failed"}
    h, w = bgr.shape[:2]
    out = {"ok": True, "w": w, "h": h}
    t0 = time.time()
    if req.get("seg"):
        try:
            out["seg"] = run_seg(bgr)
        except Exception as e:
            out["seg"] = {"err": "%s: %s" % (type(e).__name__, e)}
    if req.get("gdino"):
        try:
            out["gdino"] = run_gd(bgr, req.get("prompts") or "window. glass window. glass door.",
                                  short=int(req.get("short") or 400))
        except Exception as e:
            out["gdino"] = {"err": "%s: %s" % (type(e).__name__, e)}
    out["ms"] = round((time.time() - t0) * 1000, 1)
    return out


def main():
    _log("worker 启动", sys.version.split()[0], sys.executable)
    # 把自己的 PID 写下来 —— 控制台被直接关掉时子进程会残留，
    # 下次启动靠这个文件精确杀掉，免得两个 worker 抢 CPU（2026-11-24）
    try:
        Path(os.environ.get("CPMV_WINSEG_PIDFILE") or
             (Path(os.environ.get("TEMP", ".")) / "cpmv_winseg.pid")).write_text(str(os.getpid()))
    except Exception as e:
        _log("写 pid 文件失败:", e)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception as e:
            _REAL.write(json.dumps({"ok": False, "err": "bad json: %s" % e}) + "\n")
            _REAL.flush()
            continue
        rid = req.get("id")
        try:
            resp = handle(req)
        except Exception as e:
            resp = {"ok": False, "err": "%s: %s" % (type(e).__name__, e)}
        resp["id"] = rid
        _REAL.write(json.dumps(resp, ensure_ascii=True) + "\n")
        _REAL.flush()
        if resp.get("bye"):
            break


if __name__ == "__main__":
    main()
