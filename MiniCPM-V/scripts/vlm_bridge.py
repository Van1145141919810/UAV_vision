# -*- coding: utf-8 -*-
"""vlm_bridge.py —— 实时"第二意见"桥：把当前候选丢给 VLM，打印建议

只做建议，绝不发指令（用户要求：人来按数字 + T）。

数据来源（都在笔记本本地，不需要额外权限）：
  · 候选：GET http://127.0.0.1:8080/api/state  → gate.cands[*] = {id, uv, score, d_px}
    （8080 是飞机上的监控网页，经 SSH 隧道转到本地）
  · 图像：GET http://127.0.0.1:8091/snapshot.jpg   （gcs_gate 的预览服务）
    uv 是"检测工作分辨率"坐标（--scale 1.0 → 1920），预览默认 960 宽 → 按比例换算

触发规则（可按需调）：
  · 候选数 ≥ 2 才问（只有一个候选没有"选哪个"的问题）
  · 候选集合（id 列表）变了才问，最小间隔 --min-interval 秒
  · --only-close 时只在前两名分数差 ≤ --gap 时才问（省钱省时间）

用法：
  python scripts/vlm_bridge.py
  python scripts/vlm_bridge.py --only-close --gap 0.15
  python scripts/vlm_bridge.py --once          # 只问一次就退出（调试）
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.pick_core import suggest          # noqa: E402
from mcpv.cv import imwrite_unicode         # noqa: E402  （中文路径必须用它）


def _get_json(url: str, timeout: float = 6.0) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def _get_bytes(url: str, timeout: float = 6.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def fetch_cands(state_url: str):
    """→ (cands, stage, sim)；cands 按分数降序，只保留有像素坐标的。"""
    st = _get_json(state_url)
    g = st.get("gate") or {}
    out = []
    for c in (g.get("cands") or []):
        uv = c.get("uv")
        if not uv or len(uv) < 2:
            continue
        out.append({"id": c.get("id"), "u": float(uv[0]), "v": float(uv[1]),
                    "ps": float(c.get("score") or 0.0),
                    "d_px": float(c.get("d_px") or 0.0)})
    out.sort(key=lambda q: -q["ps"])
    return out, g.get("stage"), g.get("sim")


class Overlay(object):
    """把最新一帧（带候选编号 + VLM 建议）通过小 HTTP 服务发出去。

    不用改他的前端：把网页「识别画面」的地址改成
        http://127.0.0.1:8092/snapshot.jpg
    就能在同一块画面里看到编号和建议，不用多开窗口。
    """
    def __init__(self, port):
        self.port = port
        self.jpeg = None
        self.lock = threading.Lock()
        self.info = "(等候选…)"

    def put(self, jpeg, info):
        with self.lock:
            self.jpeg, self.info = jpeg, info

    def start(self):
        holder = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                with holder.lock:
                    buf = holder.jpeg
                if self.path.startswith("/snapshot.jpg") and buf:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(buf)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(buf)
                else:
                    body = ("vlm_bridge 预览\n%s\n\n"
                            "把网页「识别画面」的地址改成 "
                            "http://127.0.0.1:%d/snapshot.jpg\n"
                            % (holder.info, holder.port)).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(body)

        srv = ThreadingHTTPServer(("127.0.0.1", self.port), H)
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        return srv


def annotate(img, cands, pick, gap, elapsed):
    """在图上画候选编号 + 底部建议条（底部 34 px，不挡他画在左上角的阶段文字）。"""
    out = img.copy()
    for i, c in enumerate(cands, 1):
        col = (0, 255, 0) if pick == i else (0, 165, 255)
        r = max(9, int(c.dmaj / 2) + 4)
        pt = (int(c.u), int(c.v))
        cv2.circle(out, pt, r, col, 2)
        cv2.putText(out, str(i), (pt[0] + r + 3, pt[1] - r),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
        cv2.putText(out, str(i), (pt[0] + r + 3, pt[1] - r),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
    cv2.rectangle(out, (0, out.shape[0] - 34), (out.shape[1], out.shape[0]), (0, 0, 0), -1)
    if pick:
        txt = "VLM SUGGEST: #%d   (ps gap %.2f,  %.1fs)" % (pick, gap, elapsed)
        col = (0, 255, 0)
    else:
        txt = "VLM: no answer  (use ps ranking)   (%.1fs)" % elapsed
        col = (0, 200, 255)
    cv2.putText(out, txt, (8, out.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
    return out


class Cand(object):
    """给 pick_core 用的轻量候选（它只读 u/v/dmaj）。"""
    __slots__ = ("u", "v", "dmaj", "ps")

    def __init__(self, u, v, d, ps=0.0):
        self.u, self.v, self.dmaj, self.ps = u, v, max(6.0, d), ps


def main():
    ap = argparse.ArgumentParser(description="VLM 第二意见桥（只打印建议，不发指令）")
    ap.add_argument("--state", default="http://127.0.0.1:8080/api/state")
    ap.add_argument("--snap", default="http://127.0.0.1:8091/snapshot.jpg")
    ap.add_argument("--det-width", type=int, default=1920,
                    help="检测工作分辨率宽度（--scale 1.0 时 1920，0.5 时 960）")
    ap.add_argument("--min-interval", type=float, default=3.0)
    ap.add_argument("--only-close", action="store_true",
                    help="只在第一名和第二名分数差 <= --gap 时才问")
    ap.add_argument("--gap", type=float, default=0.15)
    ap.add_argument("--max-cands", type=int, default=6)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--save-dir", default=str(ROOT / "eval" / "vlm_out"))
    ap.add_argument("--serve-port", type=int, default=8092,
                    help="把带建议的画面发到 http://127.0.0.1:<port>/snapshot.jpg（0=不发）")
    a = ap.parse_args()

    save = Path(a.save_dir)
    save.mkdir(parents=True, exist_ok=True)
    tile = 220

    print(f"[vlm] 候选来源 {a.state}")
    print(f"[vlm] 图像来源 {a.snap}   检测宽度 {a.det_width}")
    print(f"[vlm] 间隔 ≥{a.min_interval}s   "
          f"{'只在分数接近时问 (gap<=%g)' % a.gap if a.only_close else '有多个候选就问'}")
    ov = None
    if a.serve_port:
        ov = Overlay(a.serve_port)
        ov.start()
        print("[vlm] 画面输出：把网页「识别画面」地址改成 "
              "http://127.0.0.1:%d/snapshot.jpg" % a.serve_port)
    print("[vlm] 只给建议，不发指令。人选数字 + T。\n")

    last_key = None
    last_t = 0.0
    n_ask = 0

    while True:
        try:
            cands, stage, sim = fetch_cands(a.state)
        except Exception as e:
            print(f"[vlm] 取状态失败（网页/隧道没开？）: {e}")
            if a.once:
                return 2
            time.sleep(2.0)
            continue

        if len(cands) < 2:
            if a.once:
                print(f"[vlm] 现在只有 {len(cands)} 个候选，没有'选哪个'的问题")
                return 0
            time.sleep(1.0)
            continue

        key = tuple(c["id"] for c in cands[:a.max_cands])
        gap = cands[0]["ps"] - (cands[1]["ps"] if len(cands) > 1 else 0.0)
        if a.only_close and gap > a.gap:
            if a.once:
                print(f"[vlm] 分数差距 {gap:.2f} > {a.gap}，不需要问")
                return 0
            time.sleep(1.0)
            continue
        if key == last_key and time.time() - last_t < a.min_interval:
            if a.once:
                return 0
            time.sleep(0.5)
            continue

        # 取图 + 换算坐标
        try:
            buf = _get_bytes(a.snap)
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError("解不出预览图")
        except Exception as e:
            print(f"[vlm] 取预览图失败: {e}")
            if a.once:
                return 2
            time.sleep(2.0)
            continue

        k = img.shape[1] / float(a.det_width)
        use = [Cand(c["u"] * k, c["v"] * k, c["d_px"] * k, c["ps"])
               for c in cands[:a.max_cands]]

        t0 = time.time()
        res = suggest(img, use, timeout=30, tile=tile)
        dt = time.time() - t0
        n_ask += 1
        stamp = time.strftime("%H%M%S")
        if res.get("grid") is not None:
            imwrite_unicode(save / ("grid_%s.jpg" % stamp), res["grid"], quality=90)

        pick = res.get("pick")
        head = (f"【VLM 建议】第 {pick} 个" if pick
                else "【VLM 建议】没给出（回退到按分数）")
        print(f"{head}")
        print(f"   候选 {len(use)} 个   分数 {'/'.join('%.2f' % c.ps for c in use)}"
              f"   间距 {gap:.2f}   阶段 {stage}   sim={sim}   用时 {dt:.1f}s")
        for i, c in enumerate(use, 1):
            mark = " ← VLM 选这个" if pick == i else ""
            print(f"     {i}. ps={c.ps:.2f}  d={c.dmaj:.0f}px  ({c.u:.0f},{c.v:.0f}){mark}")
        print(f"   回答原文: {str(res.get('answer'))[:60]}")
        print(f"   存图: {save / ('grid_%s.jpg' % stamp)}\n")

        if ov is not None:
            ann = annotate(img, use, pick, gap, dt)
            ok2, b2 = cv2.imencode(".jpg", ann,
                                  [int(cv2.IMWRITE_JPEG_QUALITY), 88])
            if ok2:
                ov.put(b2.tobytes(), "候选 %d，VLM 建议 #%s"
                       % (len(use), pick or "无"))

        last_key, last_t = key, time.time()
        if a.once:
            return 0
        time.sleep(0.5)


if __name__ == "__main__":
    sys.exit(main())
