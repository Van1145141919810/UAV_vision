# -*- coding: utf-8 -*-
"""serve_api.py —— 极简观测服务，给 Jev 那边取数据用

═══════════════════════════════════════════════════════════════════
两种交接方式，按需要选
───────────────────────────────────────────────────────────────────
方式 A（推荐，零依赖）：直接给文件
    把 eval/results/schema_sample.jsonl 拷给他，他按行读 JSON。
    优点：可复现、可审计、不需要两人同时在线。

方式 B（本脚本）：开一个 HTTP 服务，他 curl 就能拿最新观测
    优点：实时；缺点：要两人同时开着。

═══════════════════════════════════════════════════════════════════
接口（只有三个，够用就好）
───────────────────────────────────────────────────────────────────
  GET /health
        → {"ok": true, "n": 观测条数}

  GET /observations?limit=10
        → JSON 数组，返回最近 N 条观测（按时间倒序）

  GET /observations/latest
        → 单条，最新的一条

═══════════════════════════════════════════════════════════════════
用法
───────────────────────────────────────────────────────────────────
  python eval/serve_api.py                       # 默认 8899 端口
  python eval/serve_api.py --port 9000 --src 别的文件.jsonl

他那边验证：
  curl http://<你的IP>:8899/health
  curl http://<你的IP>:8899/observations?limit=3
"""
import argparse
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / "eval" / "results" / "schema_sample.jsonl"
SRC = [DEFAULT]          # 用一个列表包住，方便 handler 访问


def read_obs(limit=None):
    p = SRC[0]
    if not p.exists():
        return []
    out = []
    for line in p.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            continue
    out.reverse()                       # 最近的在前
    return out[:limit] if limit else out


class H(BaseHTTPRequestHandler):
    def _send(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")   # 方便他前端直接取
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/health":
            self._send({"ok": True, "n": len(read_obs()), "src": SRC[0].name})
        elif u.path == "/observations/latest":
            rows = read_obs(1)
            self._send(rows[0] if rows else {})
        elif u.path == "/observations":
            limit = int(q.get("limit", ["10"])[0])
            self._send(read_obs(limit))
        else:
            self._send({"error": "unknown path",
                        "try": ["/health", "/observations?limit=10",
                                "/observations/latest"]}, 404)

    def log_message(self, *a):
        pass                            # 安静点，不刷屏


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8899)
    ap.add_argument("--src", default=str(DEFAULT))
    a = ap.parse_args()
    SRC[0] = Path(a.src)
    n = len(read_obs())
    print(f"观测文件 {SRC[0]}（{n} 条）")
    print(f"监听 http://0.0.0.0:{a.port}")
    print("接口： /health   /observations?limit=N   /observations/latest")
    print("按 Ctrl-C 停止")
    HTTPServer(("0.0.0.0", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
