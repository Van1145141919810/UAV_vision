#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_monitor.py —— 飞机上的网页监控站（只读监控 + 起飞配置）
===============================================================================
在飞机上常驻一个小网页服务, 笔记本用 SSH 端口映射来看。不传视频。

  飞机:   nice -n 10 python3 flight_monitor.py      (放 tmux 里, 先起 roscore/mavros)
  笔记本: ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
              -L 8080:127.0.0.1:8080 visbot@192.168.2.20
          (ExitOnForwardFailure: 本地 8080 被占时直接退出 —— 默认 ssh 只打一行警告继续连着,
           浏览器打开的其实是旧隧道, 可能是另一架飞机)
          多台电脑同时看: 每台各开一条, 互不冲突; 同一台电脑看两架飞机: -L 8081:127.0.0.1:8080
          浏览器打开 http://localhost:8080   → 「帮助」页有完整起飞流程
  (VSCode Remote-SSH 会自动转发端口, 在"端口"面板点一下也行)

  离线试页面(不需要 ROS, 笔记本上就能跑):
          python3 flight_monitor.py --fake

  飞行脚本: 文件名 stage<版本号>_<任务>.py, 编号 = 版本, 取编号最大的。
           2026-10-30 起只有穿环 (_gate); 停靠跟踪 (_track / _real) 已删除。
           --script 指定单个脚本时只用它 (同目录 flight.py 软链接同理)。

为什么这样设计
-------------------------------------------------------------------------------
1. **独立进程**。它崩了飞行不受影响; 它只订阅话题, 从不发布任何控制量。
2. **只绑 127.0.0.1**。必须先 SSH 登录飞机才能打开; 同一 WiFi 下别人打不开,
   也就不可能误改档案。
3. **纯标准库** (http.server)。外场没网 pip 装不上; 页面也不加载任何 CDN。
4. **页面上的阈值全部来自脚本发布的 /flight_ctrl/config**, 网页里不写任何阈值。
   脚本没启动前, 用 `<飞行脚本> --dry-run --json`(代码默认值)作参考。
5. **配置页只写档案、只跑 dry-run、只生成启动脚本 (~/fly.sh), 不启动飞行**。
   飞行必须在普通 SSH 终端里启动 (运行 ~/fly.sh 或手打命令):
   脚本依赖 TTY (键盘急停 / stdin EOF 检测), 后台启动会让这些全部失效;
   飞行脚本自己也会拒绝在 tmux / VSCode 终端 / 非 TTY 里起飞。
6. 校验逻辑只有一份 —— 飞机上的飞行脚本 + flight_profile.py。
   网页只把它的 JSON 结果原样显示。

数据来源
-------------------------------------------------------------------------------
  /flight_ctrl/telemetry  脚本内部状态 (阶段/穿环段/安全余量)   5Hz
  /flight_ctrl/config     生效配置 + 派生量 + 原点 (latched)
  /rosout                 脚本的告警/事件 (按节点名过滤; 节点名从遥测里学, 不写死)
  mavros 话题              位姿/速度/电池/模式 (脚本没启动也有)
  VINS 话题               只统计频率和延迟
  ROS master              机况检查: geometric_controller、谁在发设定点、版本是否配套、
                          有没有旧的抓帧节点/旧版桥还在跑
  /vision_bridge/cam      图传桥自己报的新相机状态 (1Hz JSON: 帧率/分辨率/原样转发比例/重开次数)

节点名 /flight_ctrl 是固定的, 不随飞行脚本文件的版本号变。

单实例 / 端口 (2026-10-10)
-------------------------------------------------------------------------------
  **先占 8080, 再碰 ROS**。以前顺序反了: 第二个实例先用同名节点 init_node →
  ROS master 把**旧实例**的节点踢掉 → 新实例再绑 8080 失败退出 → 旧实例的网页还占着
  8080 但 ROS 已死, 页面"已连接"却一直"无数据"。现在第二个实例在绑端口这一步就退出,
  并打印是谁占着; 本实例的 ROS 节点被关掉时 (被同名节点顶掉 / master 退出),
  网页服务也跟着退出, 把端口让出来, 不留僵尸。

姿态 (2026-10-10)
-------------------------------------------------------------------------------
  /api/state 的 pose 里多了 roll / pitch_up (度), 给网页姿态球用。
  pitch_up 已换成航空习惯 (机头上仰为正): mavros local pose 是 ENU/FLU,
  四元数直接解出来的 pitch 是"机头向下为正"。flight_log.csv 和地面站 HUD 里的
  pitch 仍是 FLU 原始符号, 比对时差一个负号, 不是 bug。

穿环 (2026-10-19) —— 字段约定见同目录 INTERFACE_gate.md
-------------------------------------------------------------------------------
  新增数据源   /gate/status (vision_bridge 每帧 JSON, gate / gate_gcs 模式才有)
  新增接口     /api/state 里的 gate / px4 / stream_seq
               /api/stream?since=N   遥测 + 环状态的增量 (不丢点: 冲刺只有 1~2 秒,
                                     网页 5Hz 拉"最新一条"会漏掉大部分)
               /api/logs  /api/log?dir=&file=   只读飞行日志 (网页「回放」页)
  机况检查     任务 / 图传配套、环识别、PX4 机动参数 (MPC_TILTMAX_AIR 等,
               只在飞控已连接且**未解锁**时读 mavros 的参数缓存)
  布局         穿环布局 (2026-10-30 停靠跟踪布局已删除)
  离线试       python3 flight_monitor.py --fake --fake-task gate
               (带一个假的识别画面 /fake/snapshot.jpg, 用来核对画面叠加的投影)
===============================================================================
"""

import argparse
import errno
import json
import os
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 2026-10-30 第 5 步: 除 HTTP 处理 / 绑端口 / main 以外都搬到 vb09/monitor/ (原样)
from vb09.monitor.checks import paths_info, run_checks  # noqa: E402
from vb09.monitor.common import (HERE, SCRIPT_NODE, STATUS_TOPIC, _abs_node, clean,  # noqa: E402
                                 discover_scripts, fp, task_of_script)
from vb09.monitor.fake import FAKE_CAM, start_fake_gate  # noqa: E402
from vb09.monitor.launcher import (NAME_RE, _tilde, default_config, dry_run, launcher_status,  # noqa: E402
                                   normalize_knobs, normalize_sets, regen_launcher, save_site, script_cmd,
                                   temp_launcher)
from vb09.monitor.ros import start_ros  # noqa: E402
from vb09.monitor.store import STORE, list_logs, read_log  # noqa: E402

# 网页 (2026-10-30 第 5 步: monitor.html 拆成 vb09/web/ 下的几个静态文件)。只服务这张白名单里的文件。
WEB_DIR = os.path.join(HERE, "vb09", "web")
_JS = "application/javascript; charset=utf-8"
WEB_FILES = {"monitor.css": "text/css; charset=utf-8", "core.js": _JS, "kinds/ring.js": _JS, "live.js": _JS,
             "views/monitor.js": _JS, "views/detail.js": _JS, "views/checks.js": _JS, "views/config.js": _JS,
             "views/help.js": _JS, "views/cam.js": _JS, "views/replay.js": _JS, "main.js": _JS,
             "kinds/reddot.js": _JS}


def _select_cmd(body):
    """网页选点请求 → 校验 → /flight_ctrl/select。只认三种: 轨迹号 / 像素视线 (+ 那一帧的 t_cap) / 取消。"""
    try:
        mode = str(body.get("mode"))
        if mode == "track":
            cmd = {"mode": "track", "trk_id": int(body["trk_id"])}
        elif mode == "ray":
            cmd = {"mode": "ray", "xn": float(body["xn"]), "yn": float(body["yn"]), "t_cap": float(body["t_cap"])}
            if not (abs(cmd["xn"]) < 3 and abs(cmd["yn"]) < 3):
                raise ValueError("坐标超范围")
        elif mode == "none":
            cmd = {"mode": "none"}
        else:
            raise ValueError("mode 只能是 track / ray / none")
    except (KeyError, TypeError, ValueError) as e:
        return {"ok": False, "error": "选点请求不对: %s" % e}
    from vb09.monitor import ros as _ros
    ok, msg = _ros.publish_select(cmd)
    return {"ok": ok, "msg" if ok else "error": msg, "cmd": cmd}


# =============================================================================
# HTTP
# =============================================================================
class Handler(BaseHTTPRequestHandler):
    server_version = "flight_monitor/1.0"
    args = None

    def log_message(self, fmt, *a):          # 不刷屏
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8", extra=None):
        if not isinstance(body, (bytes, bytearray)):
            body = json.dumps(clean(body), ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n > 65536:
            raise ValueError("请求太大")
        raw = self.rfile.read(n) if n else b"{}"
        return json.loads(raw.decode("utf-8") or "{}")

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        a = self.args
        try:
            if u.path in ("/", "/index.html"):
                with open(os.path.join(WEB_DIR, "monitor.html"), "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if u.path.lstrip("/") in WEB_FILES:            # 白名单: 只认这几个固定文件名
                with open(os.path.join(WEB_DIR, u.path.lstrip("/")), "rb") as f:
                    return self._send(200, f.read(), WEB_FILES[u.path.lstrip("/")])
            if u.path == "/api/state":
                return self._send(200, STORE.snapshot())
            if u.path == "/api/config":
                return self._send(200, STORE.get_config())
            if u.path == "/api/events":
                since = int((q.get("since") or ["0"])[0])
                return self._send(200, {"events": STORE.get_events(since)})
            if u.path == "/api/checks":
                return self._send(200, {"checks": run_checks(a), "t": time.time()})
            if u.path == "/api/paths":
                return self._send(200, paths_info(a))
            if u.path == "/api/stream":
                try:
                    since = int((q.get("since") or ["0"])[0])
                except ValueError:
                    since = 0
                return self._send(200, STORE.stream(since))
            if u.path == "/api/logs":
                return self._send(200, list_logs(a))
            if u.path == "/api/log":
                code, body = read_log(a, (q.get("dir") or [""])[0], (q.get("file") or [""])[0])
                if isinstance(body, (bytes, bytearray)):
                    return self._send(code, body, "text/plain; charset=utf-8")
                return self._send(code, body)
            if u.path == "/fake/snapshot.jpg":
                r = FAKE_CAM.get("render")
                if STORE.mode != "fake" or r is None:
                    return self._send(503, b"fake camera only with --fake --fake-task gate",
                                      "text/plain; charset=utf-8")
                img, meta = r()
                return self._send(200, img, "image/svg+xml", {
                    "X-Frame-Age": "%.3f" % meta.pop("_age", 0.0),
                    "X-Det": json.dumps(clean(meta), ensure_ascii=True, separators=(",", ":"))})
            if fp is None and u.path.startswith(("/api/schema", "/api/site")):
                return self._send(503, {"ok": False, "error": "flight_profile.py 不在"})
            if u.path == "/api/schema":
                return self._send(200, fp.schema_json())
            if u.path == "/api/launcher":
                return self._send(200, launcher_status(a))
            if u.path == "/api/sites":
                d = os.path.expanduser(a.sites_dir)
                items = []
                bad = []
                for n in fp.list_sites(a.sites_dir):
                    p = os.path.join(d, n + ".conf")
                    # ★2026-10-21 一个读不了的 (断开的软链接 / 刚被删 / 没权限) 不能让整个列表报错
                    try:
                        items.append({"name": n, "mtime": os.path.getmtime(p)})
                    except OSError as e:
                        bad.append({"name": n, "error": "%s: %s" % (type(e).__name__, e.strerror or e)})
                return self._send(200, {"sites": items, "dir": d, "bad": bad,
                                        "script": os.path.basename(a.script or ""),
                                        "script_cmd": script_cmd(a),
                                        "task": a.task,
                                        "scripts": {t: {"name": os.path.basename(sp),
                                                        "cmd": script_cmd(a, script=sp)}
                                                    for t, sp in a.scripts.items()},
                                        "launcher": _tilde(os.path.expanduser(a.launcher)),
                                        "monitor_dir": HERE})
            if u.path == "/api/site":
                name = (q.get("name") or [""])[0]
                if not NAME_RE.match(name):
                    return self._send(400, {"ok": False, "error": "档案名不合法"})
                knobs, sets, path, errs = fp.load_site(name, a.sites_dir)
                text = ""
                if os.path.exists(path):
                    with open(path, encoding="utf-8") as f:
                        text = f.read()
                return self._send(200, {"ok": not errs, "name": name, "path": path,
                                        "knobs": knobs or {}, "sets": sets or [],
                                        "errors": errs, "text": text})
            return self._send(404, {"ok": False, "error": "not found"})
        except BrokenPipeError:
            pass
        except Exception as e:
            try:
                self._send(500, {"ok": False, "error": "%s: %s" % (type(e).__name__, e)})
            except Exception:
                pass

    def do_POST(self):
        u = urlparse(self.path)
        a = self.args
        try:
            if u.path == "/api/select":                    # 网页选点 (2026-11-08, PLAN_dot_v2 §4)
                return self._send(200, _select_cmd(self._body()))
            if fp is None:
                return self._send(503, {"ok": False, "error": "flight_profile.py 不在"})
            body = self._body()
            if u.path == "/api/preview":
                try:
                    knobs = normalize_knobs(body.get("knobs") or {})
                    sets = normalize_sets(body.get("sets"))
                except ValueError as e:
                    return self._send(200, {"ok": False, "errors": [str(e)]})
                return self._send(200, dry_run(a, knobs, sets))
            if u.path == "/api/save":
                code, res = save_site(a, body)
                return self._send(code, res)
            if u.path == "/api/launcher":
                code, res = regen_launcher(a, body)
                return self._send(code, res)
            if u.path == "/api/launcher_temp":
                code, res = temp_launcher(a, body)
                return self._send(code, res)
            return self._send(404, {"ok": False, "error": "not found"})
        except BrokenPipeError:
            pass
        except Exception as e:
            try:
                self._send(500, {"ok": False, "error": "%s: %s" % (type(e).__name__, e)})
            except Exception:
                pass


# =============================================================================
# main
# =============================================================================
def bind_or_die(args):
    """
    先占端口。被占 → 打印是谁占着, 退出 (不碰 ROS, 不会把正在运行的那个实例顶掉)。
    """
    try:
        srv = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as e:
        if e.errno != errno.EADDRINUSE:
            raise
        print("[monitor] ✗ 端口 %d 已被占用 —— 多半是上一个 flight_monitor 还活着"
              " (另一个 tmux 窗口 / 会话?)" % args.port)
        try:
            out = subprocess.run(["ss", "-ltnp", "sport = :%d" % args.port],
                                 capture_output=True, text=True, timeout=3).stdout.strip()
            if out:
                print("    占用者:\n" + "\n".join("      " + l for l in out.splitlines()))
        except Exception:
            pass
        print("    找它:  tmux ls   →  进那个窗口 Ctrl+C")
        print("    或者:  pkill -f flight_monitor.py   (只会结束监控, 不影响飞行脚本)")
        print("    确实要两个并存 (比如一个 --fake): 另一个加 --port 8081")
        sys.exit(1)
    srv.daemon_threads = True
    return srv


def main():
    ap = argparse.ArgumentParser(description="飞机上的网页监控站")
    ap.add_argument("--host", default="127.0.0.1",
                    help="监听地址 (默认只本机, 通过 SSH 端口映射访问)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--script", default=None,
                    help="只用这一个飞行脚本 (默认: 同目录编号最大的 stageNN_mission.py / stageNN_gate.py; "
                         "有 flight.py 软链接就只用它)")
    ap.add_argument("--task", choices=("auto", "gate"), default="auto",
                    help="档案没写任务时用哪种任务的脚本 (只剩 gate)")
    ap.add_argument("--sites-dir", default=None,
                    help="现场档案目录 (默认: flight_profile.DEFAULT_SITES_DIR)")
    ap.add_argument("--node", default=SCRIPT_NODE,
                    help="飞行脚本节点名 (脚本启动后以遥测里的为准)")
    ap.add_argument("--telemetry-topic", default=SCRIPT_NODE + "/telemetry")
    ap.add_argument("--config-topic", default=SCRIPT_NODE + "/config")
    ap.add_argument("--status-topic", default=STATUS_TOPIC,
                    help="vision_bridge 应订阅的状态话题 (只用于版本配套检查)")
    ap.add_argument("--cam-status-topic", default="/vision_bridge/cam",
                    help="图传桥报相机状态的话题 (1Hz JSON)")
    ap.add_argument("--det-image-topic", default="/det_cam/image_raw",
                    help="旧抓帧节点的话题: 有人发 = 它还在跑, 会和桥抢相机 (本程序不订它)")
    ap.add_argument("--old-image-topic", default="/stereo_cam/left/image_raw",
                    help="老相机图像话题: 桥订它 = 旧版桥")
    ap.add_argument("--gate-status-topic", default="/gate/status",
                    help="环识别状态 (vision_bridge gate / gate_gcs 模式每帧 JSON)")
    ap.add_argument("--log-root", default="~/uav_logs",
                    help="飞行日志根目录 (网页「回放」只读这里, 默认 ~/uav_logs)")
    ap.add_argument("--px4-refresh", type=float, default=30.0,
                    help="未解锁时多久重读一次 PX4 机动参数 (秒, 默认 30)")
    ap.add_argument("--launcher", default="~/fly.sh",
                    help="网页生成的启动脚本放哪 (默认 ~/fly.sh)")
    ap.add_argument("--launcher-max-age", type=float, default=60.0,
                    help="启动脚本生成超过多少秒, 运行时要求输入 yes 确认 (默认 60)")
    ap.add_argument("--nice", type=int, default=10, help="降低进程优先级 (默认 10)")
    ap.add_argument("--fake", action="store_true", help="不连 ROS, 用模拟数据试页面")
    ap.add_argument("--fake-task", choices=("gate",), default="gate",
                    help="--fake 时模拟哪种任务 (只剩 gate = 穿环, 带假识别画面)")
    ap.add_argument("--fake-kind", choices=("ring", "reddot"), default="ring",
                    help="--fake 时识别结果按哪种目标 (reddot = 加上红点的候选 / 选点 / 抵近字段, 看红点区和画面点选)")
    args = ap.parse_args()
    # 记下每个路径是怎么来的 (「机况检查 → 文件与路径」里显示)
    # ★2026-10-19 飞行脚本按任务找 (见 discover_scripts)
    one = args.script or (os.path.join(HERE, "flight.py")
                          if os.path.exists(os.path.join(HERE, "flight.py")) else None)
    if one:
        one = os.path.abspath(one)
        t = task_of_script(one) or "gate"
        args.scripts = {t: one}
        args.script_src = "来源: --script 参数" if args.script else "来源: 同目录的 flight.py"
    else:
        args.scripts = discover_scripts()
        args.script_src = "来源: 同目录 stage<版本>_<任务>.py 编号最大的"
    if args.task == "auto":
        args.task = "gate"
        args.task_src = "--task auto"
    else:
        args.task_src = "--task 参数"
    args.script = args.scripts.get(args.task)
    if args.sites_dir:
        args.sites_src = "--sites-dir 参数"
    elif fp is not None:
        args.sites_dir = fp.DEFAULT_SITES_DIR
        args.sites_src = "flight_profile.DEFAULT_SITES_DIR (飞行脚本不带 --sites-dir 时也用它)"
    else:
        args.sites_dir = os.path.expanduser("~/flight_sites")
        args.sites_src = "flight_profile.py 不在, 退回 ~/flight_sites"
    STORE.script_node = _abs_node(args.node)
    Handler.args = args

    if args.nice:
        try:
            os.nice(args.nice)
        except (AttributeError, OSError):
            pass
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print("[monitor] ⚠️ 监听 %s —— 同网段的人都能打开并写档案。"
              "正常请用默认 127.0.0.1 + SSH 端口映射。" % args.host)
    if fp is None:
        print("[monitor] ⚠️ 找不到 flight_profile.py —— 配置页不可用")

    # ① 先占端口: 第二个实例在这里就退出, 不会用同名节点把正在运行的那个顶掉
    srv = bind_or_die(args)
    # ② 再做耗时的 dry-run 和 ROS 初始化 (这期间网页已能打开, 只是数据还没来)
    STORE.config_fallback = default_config(args)
    if args.fake:
        start_fake_gate(args)
    else:
        try:
            start_ros(args, srv)
        except ImportError as e:
            print("[monitor] 没有 ROS 环境 (%s)。想离线试页面请加 --fake" % e)
            srv.server_close()
            sys.exit(1)

    print("[monitor] http://%s:%d  (%s)  脚本 %s  默认任务 %s  档案目录 %s"
          % (args.host, args.port, "FAKE" if args.fake else "ROS",
             "  ".join("%s=%s" % (t, os.path.basename(p)) for t, p in sorted(args.scripts.items()))
             or "(没找到)", args.task, args.sites_dir))
    if not args.fake:
        print("[monitor] 笔记本: ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 "
              "-L %d:127.0.0.1:%d visbot@<飞机IP>  然后打开 "
              "http://localhost:%d" % (args.port, args.port, args.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
