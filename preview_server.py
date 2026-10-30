# -*- coding: utf-8 -*-
"""
preview_server.py —— 识别端的画面预览服务（给飞机监控页嵌入识别画面用）
===============================================================================
只依赖 OpenCV + Python 标准库, Windows / Linux 都能用。
和 gcs_aruco_det2.py 放**同一个目录**即可 (Python 会从脚本所在目录找它)。

接进识别程序 (gcs_aruco_det2.py 已经接好):

    import preview_server
    pv = preview_server.start()            # 程序启动时调一次; 端口被占返回 None, 不抛异常

    ...                                    # 每帧: 检测 → 画框
    preview_server.publish(vis, meta={...}, raw=jpg_bytes)

  frame/vis = 画好框的检测图 (去畸变灰度图 + 框 + 坐标轴, 不画 HUD)
  meta      = 本帧检测数据, 放进响应头 X-Det (JSON), 和画面严格同一帧
  raw       = 飞机发来的**原始 JPEG** (可选)。网页选「原始」时原样转发, 零重编码 ——
              能看到的最清楚的画面就是它 (但没有检测框, 是相机原图)

画质 (网页「画质」下拉框会带这些参数, 也可以直接在浏览器里试):
    /snapshot.jpg                 默认 (start 的 max_width / quality)
    /snapshot.jpg?w=1280&q=85     高清: 缩到 1280 宽, JPEG 质量 85
    /snapshot.jpg?w=0&q=92        全尺寸: 检测图原尺寸 (1920), 带框
    /snapshot.jpg?src=raw         原始: 飞机发来的 JPEG 原样转发, 彩色, 无框
  不同参数各自缓存 (同一帧同一参数只编码一次), 不看就零开销。

地址:
    浏览器和识别程序在同一台电脑:  http://127.0.0.1:8091/snapshot.jpg   (默认)
    浏览器在另一台电脑:            start(host="0.0.0.0"),
                                   地址填 http://<识别机IP>:8091/snapshot.jpg

设计要点（每条都是为了不和传图抢带宽 / 不拖慢识别）
-------------------------------------------------------------------------------
1. publish() 只存引用, 不编码 —— 识别循环里几乎零开销。
2. 有人来要图时才编码 (在 HTTP 线程里, cv2 编码时释放 GIL, 不卡检测)。
3. 网页"看完一张再要下一张", 链路慢了自动降速, 不会堆积。
4. 响应头带 X-Frame-Age (这一帧离现在多久), 网页据此标出"画面卡住了"。
5. 默认只绑 127.0.0.1 —— 同一个 AP 上的其他人看不到。
6. 端口被占不会让识别程序崩: start() 打印原因后返回 None, 检测照常。
   Windows 上用独占绑定 (SO_EXCLUSIVEADDRUSE): 默认的 SO_REUSEADDR 在 Windows 上
   允许两个进程同时监听同一端口, 网页可能拿到旧进程冻住的画面。

自测（不需要识别程序, 生成一个会动的测试画面）:
    python3 preview_server.py --demo           然后浏览器打开 http://127.0.0.1:8091/
===============================================================================
"""

import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

_lock = threading.Lock()
_frame = None          # 最近一次 publish 的图 (BGR ndarray) 或现成的 JPEG bytes
_is_jpeg = False
_raw = None            # 飞机发来的原始 JPEG bytes (src=raw 时原样转发)
_meta = None           # 和 _frame 同帧的检测数据 (dict), 放进响应头 X-Det
_seq = 0               # 帧序号, publish 一次 +1
_t = 0.0               # publish 时刻 (time.time())
_cache = {}            # (max_width, quality) → (seq, jpeg)  —— 同一帧同一画质只编码一次
_CACHE_MAX = 4
_cfg = {"max_width": 640, "quality": 70}
_server = None


def _jsonable(x):
    """nan/inf → None, numpy 标量 → float。浏览器的 JSON.parse 不认 NaN。"""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if x is None or isinstance(x, (bool, int, str)):
        return x
    try:
        return _jsonable(float(x))
    except Exception:
        return str(x)


def publish(frame=None, jpeg=None, meta=None, raw=None):
    """
    交一帧给预览。frame / jpeg 二选一:
      frame = 画好框的 BGR 图 (np.ndarray)  —— 最常用
      jpeg  = 已经编码好的 JPEG bytes
    meta = 本帧的检测数据 (dict, 可选), 网页显示用。
    raw  = 飞机发来的原始 JPEG bytes (可选), 网页选「原始」时原样转发。
    只存引用, 不拷贝不编码。如果调用之后还会在同一个数组上继续画, 请传 frame.copy()。
    """
    global _frame, _is_jpeg, _raw, _meta, _seq, _t
    if frame is None and jpeg is None:
        return
    with _lock:
        _frame, _is_jpeg = (jpeg, True) if jpeg is not None else (frame, False)
        _raw = raw
        _meta = meta
        _seq += 1
        _t = time.time()


def _encode_latest(max_width=None, quality=None, src=None):
    """
    返回 (jpeg_bytes, seq, age_s, meta) 或 None。只在有人请求时调用。
    src="raw" → 飞机原始 JPEG; 否则按 (max_width, quality) 编码检测图 (0 = 不缩放)。
    """
    with _lock:
        frame, is_jpeg, raw, seq, t, meta = _frame, _is_jpeg, _raw, _seq, _t, _meta
    if frame is None:
        return None
    if src == "raw":
        if raw is None:
            return None
        data = raw if isinstance(raw, bytes) else bytes(raw)
        return data, seq, time.time() - t, meta

    mw = _cfg["max_width"] if max_width is None else max_width
    q = _cfg["quality"] if quality is None else quality
    key = (mw, q)
    with _lock:
        hit = _cache.get(key)
    if hit is not None and hit[0] == seq:
        return hit[1], seq, time.time() - t, meta

    if is_jpeg:
        data = bytes(frame)
    else:
        img = frame
        h, w = img.shape[:2]
        if mw and w > mw:
            img = cv2.resize(img, (mw, int(round(h * mw / w))), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(q)])
        if not ok:
            return None
        data = buf.tobytes()
    with _lock:
        _cache[key] = (seq, data)
        while len(_cache) > _CACHE_MAX:              # 只留最近几种画质
            _cache.pop(next(iter(_cache)))
    return data, seq, time.time() - t, meta


def _parse_quality(query):
    """?w=&q=&src= → (max_width|None, quality|None, src|None), 越界的值钳住。"""
    qs = parse_qs(query)

    def num(name, lo, hi):
        try:
            v = int(float(qs[name][0]))
        except (KeyError, ValueError, IndexError):
            return None
        return v if v == 0 and name == "w" else max(lo, min(hi, v))

    src = (qs.get("src") or [None])[0]
    return num("w", 160, 3840), num("q", 30, 95), ("raw" if src == "raw" else None)


_INDEX = """<!doctype html><meta charset="utf-8"><title>preview</title>
<body style="margin:0;background:#111;color:#aaa;font:13px sans-serif">
<div style="padding:6px">画质:
 <a href="?" style="color:#8cf">默认</a> · <a href="?w=1280&q=85" style="color:#8cf">高清</a> ·
 <a href="?w=0&q=92" style="color:#8cf">全尺寸</a> · <a href="?src=raw" style="color:#8cf">原始</a></div>
<img id="i" style="max-width:100%;display:block"><div id="s" style="padding:6px"></div>
<pre id="m" style="padding:0 6px;margin:0;white-space:pre-wrap"></pre>
<script>
const i=document.getElementById('i'),s=document.getElementById('s'),m=document.getElementById('m');
const qs=location.search?location.search.slice(1)+'&':'';
async function tick(){try{const r=await fetch('snapshot.jpg?'+qs+'n='+Date.now(),{cache:'no-store'});
if(r.ok){const b=await r.blob();const u=URL.createObjectURL(b);
i.onload=()=>{URL.revokeObjectURL(u);s.textContent='帧龄 '+(1000*r.headers.get('X-Frame-Age')).toFixed(0)+
' ms  #'+r.headers.get('X-Frame-Seq')+'  '+i.naturalWidth+'×'+i.naturalHeight+'  '+(b.size/1024).toFixed(0)+' KB';};
i.src=u;m.textContent=r.headers.get('X-Det')||'';}
else s.textContent='还没有画面 ('+r.status+')';}catch(e){s.textContent='连不上';}setTimeout(tick,200);}
tick();</script>""".encode("utf-8")


class _Handler(BaseHTTPRequestHandler):
    server_version = "preview_server/1.2"

    def log_message(self, *a):
        pass

    def _head(self, code, ctype, n, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(n))
        self.send_header("Cache-Control", "no-store")
        # 监控页的源是 http://localhost:8080 (飞机, 经 SSH 隧道), 这里是另一个源 → 要放行 CORS
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Expose-Headers", "X-Frame-Age, X-Frame-Seq, X-Det")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()

    def do_GET(self):
        u = urlparse(self.path)
        try:
            if u.path in ("/", "/index.html"):
                self._head(200, "text/html; charset=utf-8", len(_INDEX))
                self.wfile.write(_INDEX)
                return
            if u.path == "/snapshot.jpg":
                mw, q, src = _parse_quality(u.query)
                r = _encode_latest(mw, q, src)
                if r is None:
                    body = (b"no raw frame (detector did not pass raw=)" if src == "raw"
                            else b"no frame yet")
                    self._head(503, "text/plain", len(body))
                    self.wfile.write(body)
                    return
                data, seq, age, meta = r
                extra = {"X-Frame-Age": "%.3f" % age, "X-Frame-Seq": str(seq)}
                if meta is not None:
                    try:   # ensure_ascii: HTTP 头只能是 latin-1, 中文会被转成 \\uXXXX
                        extra["X-Det"] = json.dumps(_jsonable(meta), ensure_ascii=True,
                                                    separators=(",", ":"))
                    except Exception:
                        pass
                self._head(200, "image/jpeg", len(data), extra)
                self.wfile.write(data)
                return
            self._head(404, "text/plain", 0)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    # Windows 上 SO_REUSEADDR = "允许两个进程同时监听同一端口", 请求由谁接不确定。
    # 所以 Windows 上不用它, 改为独占绑定; Linux 上保留 (只影响 TIME_WAIT, 无害)。
    allow_reuse_address = (os.name != "nt")

    def server_bind(self):
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def start(port=8091, host="127.0.0.1", max_width=640, quality=70):
    """
    起一个后台线程提供 /snapshot.jpg。重复调用无副作用。
    host="0.0.0.0" 才能让别的电脑来看 (Windows 第一次会弹防火墙提示)。
    max_width / quality 只是**默认**画质; 网页可以用 ?w=&q=&src=raw 按次要更清楚的。
    端口被占 → 打印原因, 返回 None (不抛异常: 预览是可选的, 不能连累检测)。
    """
    global _server
    _cfg["max_width"], _cfg["quality"] = max_width, quality
    if _server is not None:
        return _server
    try:
        srv = _Server((host, port), _Handler)
    except OSError as e:
        print("[preview] ⚠ 端口 %d 绑定失败 (%s) —— 网页预览关闭, 检测照常运行" % (port, e))
        if os.name == "nt":
            print("          查占用: netstat -ano | findstr :%d   再 tasklist /fi \"pid eq <PID>\""
                  % port)
        else:
            print("          查占用: ss -ltnp 'sport = :%d'" % port)
        print("          多半是上一个检测程序还开着; 或换端口: --preview-port 8092")
        return None
    threading.Thread(target=srv.serve_forever, daemon=True,
                     name="preview_server").start()
    _server = srv
    print("[preview] http://%s:%d/snapshot.jpg  (默认缩放到 %s 宽, 质量 %d; 网页可选高清/原始)"
          % ("127.0.0.1" if host in ("127.0.0.1", "localhost") else host, port,
             max_width or "原始", quality))
    return srv


def stop():
    global _server
    if _server is not None:
        _server.shutdown()
        _server.server_close()
        _server = None


# ------------------------------------------------------------------ 自测
if __name__ == "__main__":
    import argparse
    import numpy as np

    ap = argparse.ArgumentParser(description="识别画面预览服务 (自测模式)")
    ap.add_argument("--demo", action="store_true", help="生成会动的测试画面")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--max-width", type=int, default=640)
    a = ap.parse_args()
    if start(a.port, a.host, a.max_width) is None:
        raise SystemExit(1)
    if not a.demo:
        print("没有 --demo: 只开服务, 没有画面。Ctrl+C 退出")
    t0 = time.time()
    try:
        while True:
            if a.demo:
                t = time.time() - t0
                # 1920×1080 的测试图: 细格子 + 小字, 方便比较各档画质的清晰度
                img = np.full((1080, 1920, 3), 40, np.uint8)
                for gx in range(0, 1920, 40):
                    cv2.line(img, (gx, 0), (gx, 1079), (70, 70, 70), 1)
                for gy in range(0, 1080, 40):
                    cv2.line(img, (0, gy), (1919, gy), (70, 70, 70), 1)
                for k in range(8):
                    cv2.putText(img, "fine text %d  0123456789" % k, (60, 140 + 40 * k),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45 + 0.1 * k, (200, 200, 200), 1)
                x = int(960 + 600 * np.cos(t)); y = int(540 + 360 * np.sin(1.3 * t))
                cv2.rectangle(img, (x - 80, y - 80), (x + 80, y + 80), (0, 220, 0), 3)
                cv2.putText(img, "DEMO t=%.1fs" % t, (24, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.6, (220, 220, 220), 3)
                raw = img.copy()
                cv2.putText(raw, "RAW (no boxes)", (24, 1040),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 200, 255), 2)
                ok, raw_jpg = cv2.imencode(".jpg", raw, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                found = int(t) % 5 != 4                 # 每 5 秒丢 1 秒, 看网页的 ✗ 显示
                publish(img, raw=raw_jpg.tobytes(), meta={
                    "mode": "DEMO", "probe": False, "found": found,
                    "reason": "" if found else "no marker",
                    "dist": 2.0 + 0.5 * math.sin(t) if found else None,
                    "px": 140.0 + 30 * math.sin(t) if found else None,
                    "rms": 0.4 if found else None, "conf": 0.6 if found else 0.0,
                    "fps": 10.0, "loss": 0.0, "det_ms": 35.0,
                    "rtt": 180.0 + 40 * abs(math.sin(t)), "stall": 0})
            time.sleep(0.1)          # 10Hz, 和识别帧率差不多
    except KeyboardInterrupt:
        stop()
