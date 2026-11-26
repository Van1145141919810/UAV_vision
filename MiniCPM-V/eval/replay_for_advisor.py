# -*- coding: utf-8 -*-
"""replay_for_advisor.py —— 离线回放：假装是飞机/地面站，喂 vlm_advisor

它同时开两个端口，形状和现场一致：
    :18080/api/state      {"gate": {"cands": [...], "stage": "Manual", "sim": false}}
    :18091/snapshot.jpg   校正后左目（也就是 :8091 给的那张）

--cands 0  = 假装 CV 完全没看见（远距），用来测"顾问给方向"这一路
--cands 3  = 假装 CV 已经稳住，用来测"顾问闭嘴"这一路

用法：
    python eval/replay_for_advisor.py --frame 300 --cands 0
    python scripts/vlm_advisor.py --state http://127.0.0.1:18080/api/state \
        --snap http://127.0.0.1:18091/snapshot.jpg --once 1 --save-dir %TEMP%\adv
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
S14 = Path(r"D:\Desktop\无人系统\_ref\s14")
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(S14))

from vb09.perception.stereo_rig import StereoRig            # noqa: E402
from mcpv.detect_live import detect_reds                     # noqa: E402

VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
STATE = {}


def handle_state(port, nfile=None, pool=None):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            # --n-file 里写几，就假装有几个候选（方便一个回放服务测多种状态）
            st = STATE
            if nfile:
                try:
                    n = int(open(nfile).read().strip())
                except Exception:
                    n = None
                if n is not None and pool is not None:
                    st = {"gate": {"cands": pool[:max(0, min(n, len(pool)))],
                                   "stage": "Manual", "sim": False}}
            b = json.dumps(st, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

    s = ThreadingHTTPServer(("127.0.0.1", port), H)
    threading.Thread(target=s.serve_forever, daemon=True).start()
    return s


def handle_snap(port, box):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if not self.path.startswith("/snapshot.jpg") or box.get("jpeg") is None:
                self.send_response(404)
                self.end_headers()
                return
            b = box["jpeg"]
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

    s = ThreadingHTTPServer(("127.0.0.1", port), H)
    threading.Thread(target=s.serve_forever, daemon=True).start()
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", type=int, default=300)
    ap.add_argument("--cands", type=int, default=0, help="假装有几个候选（0=CV 没看见）")
    ap.add_argument("--state-port", type=int, default=18080)
    ap.add_argument("--snap-port", type=int, default=18091)
    ap.add_argument("--n-file", default=None,
                    help="文件里写几就假装几个候选（0=CV 看不见），方便一个服务测多种状态")
    a = ap.parse_args()

    cap = cv2.VideoCapture(str(VID))
    cap.set(cv2.CAP_PROP_POS_FRAMES, a.frame)
    ok, fr = cap.read()
    cap.release()
    if not ok:
        print("读不到帧 %d" % a.frame)
        return 1

    rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
    L, _ = rig.rectify(fr, fr)

    cands = []
    pool_n = max(a.cands, 6) if a.n_file else a.cands
    if pool_n > 0:
        ds = detect_reds(L)
        ds.sort(key=lambda c: -(c["d"] or 0))
        for k, c in enumerate(ds[:pool_n]):
            cands.append({"id": k + 1, "uv": [float(c["cx"]), float(c["cy"])],
                          "score": round(0.95 - 0.05 * k, 3),
                          "d_px": float(c["d"])})
        print("造了 %d 个候选（取检测到的最大几个，只为形状对）" % len(cands))

    STATE.clear()
    STATE.update({"gate": {"cands": cands, "stage": "Manual", "sim": False}})

    ok, buf = cv2.imencode(".jpg", cv2.resize(L, (960, 540), interpolation=cv2.INTER_AREA),
                           [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    box = {"jpeg": buf.tobytes()}

    handle_state(a.state_port, a.n_file, cands)
    handle_snap(a.snap_port, box)
    print("回放服务已起（帧 %d，cands=%d）" % (a.frame, len(cands)))
    print("  http://127.0.0.1:%d/api/state" % a.state_port)
    print("  http://127.0.0.1:%d/snapshot.jpg" % a.snap_port)
    try:
        while True:
            import time
            time.sleep(3600)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
