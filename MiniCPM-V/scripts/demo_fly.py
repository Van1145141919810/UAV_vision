# -*- coding: utf-8 -*-
"""demo_fly.py —— 远距点击飞行（真飞 / dryrun），控制走 UDP。

═══════════════════════════════════════════════════════════════════
和旧版的区别（为什么重写）
───────────────────────────────────────────────────────────────────
旧版每条指令都 scp 一个文件，单程 0.3~2 秒。手动 WASD 时这是**危险**的：
你按了它不动 → 多按几次 → 突然连着动一下。
现在控制走 UDP（局域网 <5 ms），飞机侧还有 300 ms 看门狗兜底。

═══════════════════════════════════════════════════════════════════
按键
───────────────────────────────────────────────────────────────────
  鼠标单击 / 双击     选中 / 锁定（红点会跑 VLM 判标记）
  回车                发送这一步位移（机体系 dx/dy/dz，单位米）
  W A S D             前后左右（按住持续，松手 0.35 秒内自动停）
  R F                 升 / 降
  Q E                 左转 / 右转
  空格                立刻停
  L                   ★ 原地降落（AUTO.LAND）—— 现在真飞靠它收尾
  X                   退出程序，飞机保持悬停（留给以后交接文豪）
  N                   重拍一帧（不飞）
  V                   开关窗户框
  Esc                 取消选中

═══════════════════════════════════════════════════════════════════
帧延迟怎么来的
───────────────────────────────────────────────────────────────────
画面上「帧延迟 x.x s」是**飞机那侧**算的（抓图时刻到现在），
不受两台机器时钟不一致影响。正常 0.5~2.0 s（抓图 1.5 s 一帧）。
"""
import argparse
import json
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from demo_click import Demo, wheel_delta                      # noqa: E402
from mcpv.scene import describe_marker, describe_target        # noqa: E402
from mcpv.report import make_record, save_record               # noqa: E402

DRONE = "visbot@192.168.2.20"
DRONE_IP = "192.168.2.20"
PORT = 8850
KEY = str(Path.home() / ".ssh" / "dsh_drone")
FRAME_LOCAL = Path.home() / "AppData/Local/Temp/cpmv_frame.jpg"

SEQ = [0]


# ═══════════════════ 通信 ═══════════════════
def _ssh(args, timeout=25):
    return subprocess.run(["ssh", "-i", KEY, "-o", "StrictHostKeyChecking=no",
                           "-o", "ConnectTimeout=8", DRONE] + args,
                          capture_output=True, text=True, timeout=timeout)


def _scp_from(remote, local, timeout=25):
    return subprocess.run(["scp", "-i", KEY, "-o", "StrictHostKeyChecking=no",
                           f"{DRONE}:{remote}", str(local)],
                          capture_output=True, text=True, timeout=timeout)


class VideoStream:
    """TCP 长连接收帧（机上 vstream.py 推）。

    为什么不用 scp 连续拉：scp 每帧新建 SSH 连接，Windows 侧 0.5 秒/次，
    实测只有 1.5 fps。走长连接实测 10.4 fps。
    """

    def __init__(self, host=DRONE_IP, port=8890):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.latest = None
        self.count = 0
        self.alive = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _recv(self, n):
        buf = b""
        while len(buf) < n:
            b = self.sock.recv(n - len(buf))
            if not b:
                raise ConnectionError("closed")
            buf += b
        return buf

    def _loop(self):
        import numpy as np
        while self.alive:
            try:
                ln = struct.unpack(">I", self._recv(4))[0]
                if ln > 4_000_000:
                    raise ConnectionError("bad length")
                data = self._recv(ln)
            except Exception:
                self.alive = False
                return
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                self.latest = img
                self.count += 1

    def close(self):
        self.alive = False
        try:
            self.sock.close()
        except Exception:
            pass


class Link:
    """UDP 双向：指令发出去，状态收回来。

    收状态干什么：按回车后要等飞机**真的停稳**再拉新帧，
    否则拍到的是运动模糊的中间帧（固定 sleep(2.2s) 飞 3 米根本不够）。
    """

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status = None            # 执行器回传的最新状态
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", PORT + 1))     # 8851
        self.rx.settimeout(0.5)
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _rx_loop(self):
        while True:
            try:
                data, _ = self.rx.recvfrom(2048)
                self.status = json.loads(data.decode())
            except Exception:
                continue

    def send(self, **kw):
        # seq 用毫秒时间戳：两边重启都不会撞号（踩过：自增计数器会在机端重启后失效）
        SEQ[0] = max(SEQ[0] + 1, int(time.time() * 1000))
        kw["seq"] = SEQ[0]
        try:
            self.sock.sendto(json.dumps(kw).encode(), (DRONE_IP, PORT))
            return True
        except Exception as e:
            print(f"  ✗ UDP 发送失败 {e}")
            return False

    def wait_arrive(self, timeout=15.0, tol=0.20):
        """等到"离目标 < tol 米"连续 3 次（≈0.3 s）才认为停稳。"""
        t0 = time.time()
        ok = 0
        last = None
        while time.time() - t0 < timeout:
            st = self.status
            if st and st.get("dist") is not None:
                last = st["dist"]
                if last < tol:
                    ok += 1
                    if ok >= 3:
                        return True, last
                else:
                    ok = 0
            time.sleep(0.1)
        return False, last


def pull_frame():
    """拉一帧，并取回"从抓图到现在"的延迟（在飞机上算，避开时钟不同步）。"""
    t0 = time.time()
    r = _scp_from("/tmp/cpmv_frame.jpg", FRAME_LOCAL)
    if r.returncode != 0 or not FRAME_LOCAL.exists():
        print(f"  ✗ 拉帧失败: {r.stderr.strip()[:100]}")
        return None
    age = None
    try:
        out = _ssh(["date +%s.%N; cat /tmp/cpmv_frame.jpg.stamp 2>/dev/null"]).stdout.split()
        if len(out) >= 2:
            age = float(out[0]) - float(out[1])
    except Exception:
        age = None
    return {"transfer_s": time.time() - t0, "age_s": age}


# ═══════════════════ 界面 ═══════════════════
class FlyDemo(Demo):
    def __init__(self, live, link):
        super().__init__([FRAME_LOCAL])
        self.live = live
        self.link = link
        self.frame_info = None
        self.manual = (0.0, 0.0, 0.0, 0.0)
        self.manual_until = 0.0
        self.arrive = {"active": False, "ok": 0, "t0": 0.0}
        self.video_mode = False       # M 键切换：连续刷新的低分辨率画面
        self.vstream = None           # TCP 视频流（第一次按 M 时建）
        self._stop = False
        threading.Thread(target=self._manual_loop, daemon=True).start()

    def _manual_loop(self):
        """20 Hz 持续发速度；松手 0.35 s 后自动归零（机端还有 300 ms 看门狗）。"""
        while not self._stop:
            if time.time() < self.manual_until:
                vbx, vby, vz, yr = self.manual
                self.link.send(type="vel", vbx=vbx, vby=vby, vz=vz, yr=yr)
            time.sleep(0.05)

    def nudge(self, vbx, vby, vz, yr):
        self.manual = (vbx, vby, vz, yr)
        self.manual_until = time.time() + 0.35

    def refresh(self):
        info = pull_frame()
        if not info:
            return False
        self.frame_info = info
        self.files = [FRAME_LOCAL]
        self.idx = 0
        self.load()
        self.dirty = True
        return True

    def fire(self):
        if not self.sel:
            return
        if self.arrive["active"]:
            # 上一段还没飞完就再按回车 → 忽略，避免连发两条位移
            print("  ⏳ 上一段还在飞，先等它到位")
            return
        c = self.sel["cmd"]
        if c.arrived:
            print("  ⏸ 已到位，不发送")
            return
        # ★ 先发指令，再存 JSON：
        #   super().fire() 里要调 VLM（1.5~2.5 秒），如果先存再发，
        #   按下回车后要等两秒飞机才动。飞控响应比存档重要。
        self.link.send(type="step",
                       dx=round(c.forward, 3),
                       dy=round(c.left, 3),
                       dz=round(c.up, 3))
        print(f"  → UDP step 前{c.forward:+.2f} 左{c.left:+.2f} 上{c.up:+.2f}")
        # ★ 存档放后台线程：VLM 要 1.5~2.5 秒，如果同步做，
        #   这段时间画面冻住、WASD 也按不动 —— 真飞时是不能接受的。
        #   先把要用的东西**拷一份**再交给线程，避免它和 refresh() 抢 self.img。
        cu, cvv = self.sel["point"]
        half = 200
        x0, y0 = max(0, int(cu - half)), max(0, int(cvv - half))
        x1 = min(self.img.shape[1], int(cu + half))
        y1 = min(self.img.shape[0], int(cvv + half))
        patch = self.img[y0:y1, x0:x1].copy()
        if patch.size:
            patch = cv2.resize(patch, (384, 384), interpolation=cv2.INTER_LANCZOS4)
        sel = {k: v for k, v in self.sel.items() if k not in ("cmd", "armed")}
        img_path = str(self.files[self.idx % len(self.files)])
        imgsize = (self.img.shape[1], self.img.shape[0])
        kind = self.sel["kind"]
        threading.Thread(target=self._save_bg, daemon=True,
                         args=(patch, sel, c, img_path, imgsize, kind)).start()
        if self.live:
            # **非阻塞**：只登记"在等到位"，主循环继续转（画面不冻）
            self.arrive = {"active": True, "ok": 0, "t0": time.time()}

    def _save_bg(self, patch, sel, cmd, img_path, imgsize, kind):
        """后台存档：调 VLM + 写 record JSON。不碰 self.img，只用自己的副本。"""
        try:
            if patch.size:
                scene = (describe_marker(patch) if kind == "dot"
                         else describe_target(patch))
            else:
                scene = {"error": "crop empty"}
        except Exception as e:
            scene = {"error": f"{type(e).__name__}: {e}"}
        try:
            rec = make_record(img_path, imgsize, sel, cmd, scene)
            path = save_record(rec)
            print(f"  ▶ 语义(后台): {scene}")
            print(f"  ▶ 已存: {path}")
            if kind == "dot":
                v = {"A": "是标记", "B": "不是标记", "?": "没看懂"}.get(
                    scene.get("marker"), scene.get("marker"))
                self.flash = f"已发送  红点判定={v}  → {path.name}"
            else:
                self.flash = (f"已发送  窗={scene.get('window_state')} "
                              f"反光={scene.get('glare')}  → {path.name}")
        except Exception as e:
            print(f"  ✗ 存档失败: {type(e).__name__}: {e}")

    def toggle_video(self):
        """M 键：切"视频模式"（连续刷新的低分辨率画面）↔ 精细模式。

        为什么要切：YUYV 全分辨率下相机 1.5 s 才出一帧，手动飞根本没法用。
        视频模式把相机切到 MJPG + 缩一半（960×540, 120 KB），实测 ~6 fps。
        代价：边缘会糊（11 px vs 5 px），所以**精细瞄准时一定要按 M 切回来**。
        """
        self.video_mode = not self.video_mode
        mode = "video" if self.video_mode else "fine"
        r = _ssh([f"echo {mode} > /tmp/cpmv_mode"])
        if r.returncode != 0:
            print(f"  ✗ 切换失败：{r.stderr.strip()[:80]}")
            self.video_mode = not self.video_mode
            return
        print(f"\n★ M：{'视频模式（连续刷新，~6 fps，画面略糊）' if self.video_mode else '精细模式（1.5 s/帧，最清晰）'}")
        if not self.video_mode:
            # 切回精细：相机要重开（YUYV 首帧要 10 秒），先给个提示
            print("  正在切回精细模式，相机重开约 10 秒…")
            time.sleep(11)
            self.refresh()

    def video_tick(self):
        """视频模式：显示 TCP 流里最新的一帧（不做检测，只为看得见）。"""
        if self.vstream is None or not self.vstream.alive:
            try:
                self.vstream = VideoStream()
            except Exception as e:
                print(f"  ✗ 连不上视频流（vstream 起了吗？）{e}")
                self.video_mode = False
                return
        img = self.vstream.latest
        if img is None:
            return
        vw, vh = self.view_size
        view = cv2.resize(img, (vw, vh), interpolation=cv2.INTER_LINEAR)
        cv2.putText(view, "视频模式（M 切回精细）  WASD/RF/QE 手动  L 降落  X 退出",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        st = self.link.status
        if st:
            d = st.get("dist")
            line = (f"{st.get('mode', '?')} armed={st.get('armed')} 离目标 {d:.2f} m"
                    if d is not None else st.get("mode", "?"))
            cv2.putText(view, line, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (120, 255, 120) if st.get("live") else (200, 200, 120), 2)
        cv2.imshow("cpmv-fly", view)

    def tick_arrive(self):
        a = self.arrive
        if not a["active"]:
            return
        st = self.link.status
        d = st.get("dist") if st else None
        if d is not None and d < 0.20:
            a["ok"] += 1
        else:
            a["ok"] = 0
        if a["ok"] >= 3:                       # 连续 3 次（≈0.6 s）都到位才算停稳
            print(f"  ✓ 已到位（离目标 {(d or 0):.2f} m），拉新帧")
            a["active"] = False
            self.refresh()
        elif time.time() - a["t0"] > 15.0:
            print(f"  ⚠ 等位超时（离目标 {d if d is not None else '?'} m），照样拉一帧")
            a["active"] = False
            self.refresh()

    def render(self):
        view = super().render()
        if self.frame_info:
            age = self.frame_info.get("age_s")
            txt = f"帧延迟 {age:.1f}s" if age is not None else "帧延迟 ?"
            color = (0, 255, 0) if (age is not None and age < 3) else (0, 0, 255)
            cv2.putText(view, txt, (view.shape[1] - 190, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        # 执行器回传的状态：模式 / 解锁 / 离目标多远
        st = self.link.status
        if st:
            d = st.get("dist")
            line = (f"{st.get('mode', '?')}  armed={st.get('armed')}  "
                    f"离目标 {d:.2f} m" if d is not None else st.get("mode", "?"))
            cv2.putText(view, line, (10, 64), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (120, 255, 120) if st.get("live") else (200, 200, 120), 2)
        return view


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    print("=" * 62)
    print("  远距点击飞行  ——  模式？")
    print("    1 = 真飞（螺旋桨已装、场地清空、你自己盯得住）")
    print("    2 = dryrun（只算指令、只存 JSON，飞机不动）")
    print("=" * 62)
    if args.live:
        live = True
    elif args.dry:
        live = False
    else:
        live = input("选 1 或 2: ").strip() == "1"
    print(f"\n模式: {'★ 真飞' if live else 'dryrun'}    Python: {sys.executable}")

    if "OK" not in _ssh(["echo OK"]).stdout:
        print("✗ 连不上飞机")
        return 1

    if not _ssh(["pgrep -f grabd.py | head -1"]).stdout.strip():
        print("抓图进程没跑，启动中…")
        _ssh(["bash ~/cpmvbyhuang/start_grabd.sh yuyv"])
        time.sleep(20)

    link = Link()
    demo = FlyDemo(live, link)
    if not demo.refresh():
        return 1

    if live:
        print("\n⚠ 真飞：确认螺旋桨已装、周围清空、电池 >15V")
        if input("输入 YES 继续: ").strip() != "YES":
            return 0
        link.send(type="takeoff", z=1.5)
        print("  起飞已下发（机上：流2秒 → 切OFFBOARD → 解锁 → 爬升）")
        time.sleep(10)
        demo.refresh()

    print("\n双击框→回车 | WASD/RF/QE 手动 | L 降落 | X 悬停退出 | N 重拍 | Esc 取消")

    def on_mouse(event, x, y, flags, param):
        # 双击会产生 3 个事件（DOWN、DOWN、DBLCLK），不合并就会刷屏，
        # 让人看不清"到底锁没锁上"。这里按"位置+时间"去重。
        now = time.time()
        if event == cv2.EVENT_LBUTTONDOWN:
            if now - on_mouse.last[0] < 0.4 and \
               abs(x - on_mouse.last[1]) < 5 and abs(y - on_mouse.last[2]) < 5:
                return
            on_mouse.last = (now, x, y)
        try:
            if event == cv2.EVENT_LBUTTONDOWN:
                demo.select(x, y)
                demo.dirty = True
            elif event == cv2.EVENT_LBUTTONDBLCLK:
                demo.arm()
                demo.dirty = True
            elif event == cv2.EVENT_MOUSEWHEEL:
                d = wheel_delta(flags)
                if d:
                    demo.zoom_at(1.25 if d > 0 else 0.8, x, y)
                    demo.dirty = True
            elif event == cv2.EVENT_RBUTTONDOWN:
                demo.pan_anchor = (x, y)
            elif event == cv2.EVENT_MOUSEMOVE and demo.pan_anchor is not None:
                ax, ay = demo.pan_anchor
                _, _, kx, ky, _, _ = demo._tf()
                demo.center = (demo.center[0] - (x - ax) / kx,
                               demo.center[1] - (y - ay) / ky)
                demo.pan_anchor = (x, y)
                demo.dirty = True
            elif event == cv2.EVENT_RBUTTONUP:
                demo.pan_anchor = None
        except Exception:
            import traceback
            traceback.print_exc()
    on_mouse.last = (0.0, 0, 0)

    cv2.namedWindow("cpmv-fly", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("cpmv-fly", on_mouse)

    MANUAL = {
        ord("w"): (0.5, 0, 0, 0), ord("s"): (-0.5, 0, 0, 0),
        ord("a"): (0, 0.5, 0, 0), ord("d"): (0, -0.5, 0, 0),
        ord("r"): (0, 0, 0.4, 0), ord("f"): (0, 0, -0.4, 0),
        ord("q"): (0, 0, 0, 0.5), ord("e"): (0, 0, 0, -0.5),
    }
    LAND_T = [0.0]          # 第一次按 L 的时刻（再按一次才执行）

    try:
        while True:
            if demo.video_mode:
                demo.video_tick()          # 视频模式：连续刷新，不做检测
                k = cv2.waitKey(1) & 0xFF
            else:
                demo.tick_arrive()        # 非阻塞：判断飞到了没有，到了自动重拍
                if demo.dirty:
                    cv2.imshow("cpmv-fly", demo.render())
                    demo.dirty = False
                k = cv2.waitKey(20) & 0xFF

            if k == ord("x"):
                print("\n★ X：停发设定点，飞机保持悬停（留给文豪）")
                link.send(type="exit")
                break
            elif k == ord("l"):
                # ★ 不用 input()：那会把画面卡住、焦点跳到命令行，真飞时很危险。
                # 改成"3 秒内再按一次 L"确认，全程待在画面里。
                now = time.time()
                if now - LAND_T[0] < 3.0:
                    print("\n★ L 确认：原地降落 AUTO.LAND")
                    link.send(type="land")
                    break
                LAND_T[0] = now
                demo.flash = "★ 再按一次 L 确认降落（3 秒内）"
                demo.dirty = True
                print("\n★ L：再按一次 L 确认降落（3 秒内有效）")
            elif k in (13, 10):
                demo.fire()
                demo.dirty = True
            elif k == 27:
                demo.sel = None
                demo.dirty = True
            elif k == ord("n"):
                print("  拉新帧…")
                demo.refresh()
            elif k == ord("m"):
                demo.toggle_video()
            elif k == ord("v"):
                demo.show_win = not demo.show_win
                demo.dirty = True
            elif k == 32:
                demo.manual_until = 0.0
                link.send(type="vel", vbx=0, vby=0, vz=0, yr=0)
                print("  ⏹ 停")
            elif k in MANUAL:
                # 手动接管 = 放弃"自动等到位重拍"：你已经自己飞了，
                # 再把画面换成"自动判定到位"的那张会误导你。
                demo.arrive["active"] = False
                demo.nudge(*MANUAL[k])
                print(f"  手动 {MANUAL[k]}")
    finally:
        demo._stop = True
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
