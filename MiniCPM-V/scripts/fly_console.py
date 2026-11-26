# -*- coding: utf-8 -*-
"""fly_console.py —— 一站式控制台：一个命令行 + 一个网页

它干什么
──────────────────────────────────────────────────────────────
  · 独占一个 SSH 会话跑 `bash ~/fly.sh`（和以前一模一样，文豪的代码一行不改）
  · 把本地按键**原样转发**给飞机（WASD / R F / Q E / 空格 / T / L / X / 1-5 全部照旧）
  · 额外接管两个键：
        Y      问一次「窗户在哪」→ 三路融合 → 3.2 s 后出建议
        Enter  执行当前建议（往 SSH 里注入按键，走文豪手动段的速度积分）
  · 把建议写进 advice.json，gcs_gate 那边读它画到 8091 那张图上

三种运行方式
    python scripts\\fly_console.py --offline            # 桌面演练：拿录像帧当相机，不连飞机
    python scripts\\fly_console.py --live --dry-run     # 连飞机，但**不真的动**（只打印要发什么键）
    python scripts\\fly_console.py --live --allow-move  # 真的动（默认不允许，必须显式开）

安全
  · 默认 --dry-run：Enter 只打印、不注入
  · 只在 stage=manual（手动段）时才允许注入；其它阶段一律透传
  · 任何异常 → 停发 + 打印，绝不吞掉后继续发
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "eval"))

from mcpv.window_finder import WindowFinder, draw_result          # noqa: E402
from advice_overlay import draw_advice, to_advice_dict, write_advice  # noqa: E402

SSH_KEY = str(Path.home() / ".ssh" / "dsh_drone")
DRONE = "visbot@192.168.2.20"
SNAP_RAW = "http://127.0.0.1:8091/snapshot.jpg?src=raw&w=0&q=90"

# ── 按键注入参数（跟文豪的手动段对齐；改这里）──────────────────────
TELEOP_XY_SPEED = 1.6      # m/s，档案 演示优化.conf 里的值
TELEOP_Z_SPEED = 0.6       # m/s
TELEOP_YAW_RATE = 0.3      # rad/s = 17.2 °/s
DEADMAN = 0.3              # s，松键后速度归零
HZ = 20                    # 注入频率

# 一步的动作量限幅
MAX_FWD_M = 4.0
MAX_DZ_M = 2.0
MAX_YAW_DEG = 25.0


# ══════════════════════════════════════════════════════════════════
# 键盘（Windows 原生，不依赖 termios）
# ══════════════════════════════════════════════════════════════════
class Keys(object):
    def __init__(self):
        import msvcrt
        self.m = msvcrt

    def get(self):
        """返回一个字符（小写），没有则 None。"""
        if not self.m.kbhit():
            return None
        ch = self.m.getch()
        if ch in (b"\x00", b"\xe0"):       # 方向键/功能键：吃掉第二个字节
            if self.m.kbhit():
                self.m.getch()
            return "?"
        if ch in (b"\r", b"\n"):
            return "\r"
        try:
            return ch.decode("ascii", "ignore").lower()
        except Exception:
            return None


# ══════════════════════════════════════════════════════════════════
# 抢答：Y 触发的 VLM 在后台跑，别卡住键盘转发
# ══════════════════════════════════════════════════════════════════
class Advisor(object):
    def __init__(self, finder):
        self.f = finder
        self.lock = threading.Lock()
        self.busy = False
        self.advice = None
        self.last_error = None

    def ask(self, bgr):
        with self.lock:
            if self.busy:
                return False
            self.busy = True
        t = threading.Thread(target=self._run, args=(bgr,), daemon=True)
        t.start()
        return True

    def ask_async(self, grab_fn):
        """把「取一帧」也放进后台线程 —— 取图有 5 s 超时，不能卡住键盘转发。

        2026-11-24 安全修正：以前主循环里同步取图，按 I 时最长卡 5 s，
        那 5 s 里按 X / L 传不出去。现在主循环立刻返回。
        """
        with self.lock:
            if self.busy:
                return False
            self.busy = True

        def job():
            try:
                img, tag = grab_fn()
                if img is None:
                    print("\n▌ 取到空图", flush=True)
                    return
                print("\n▌ 看图（%s）…" % tag, flush=True)
                self._do(img)
            except Exception as e:
                print("\n▌ 取图失败: %s: %s" % (type(e).__name__, e), flush=True)
            finally:
                with self.lock:
                    self.busy = False
        threading.Thread(target=job, daemon=True).start()
        return True

    def _run(self, bgr):
        try:
            self._do(bgr)
        finally:
            with self.lock:
                self.busy = False

    def _do(self, bgr):
        try:
            r = self.f.find(bgr)
            H, W = bgr.shape[:2]
            d = to_advice_dict(r, W, H)
            write_advice(d)
            self.advice = dict(d, _raw=r)
            print("\n★ 建议: %s" % d.get("text", ""), flush=True)
            print("   来源 %s   %s ms   (dot %s / seg %s / gdino %s)" % (
                d.get("source"), (d.get("ms") or {}).get("total"),
                (d.get("ms") or {}).get("dot"), (d.get("ms") or {}).get("seg"),
                (d.get("ms") or {}).get("gdino")), flush=True)
            if d.get("state") != "ok":
                print("   → 看不清，不执行。请你手动挪一下再按 Y。", flush=True)
            else:
                plan = make_plan(d, bgr.shape)
                for ln in plan["lines"]:
                    print("   " + ln, flush=True)
                print("   → 按 Enter 执行，或继续自己飞", flush=True)
                self.advice["_plan"] = plan
        except Exception as e:
            self.last_error = "%s: %s" % (type(e).__name__, e)
            print("\n[顾问] 出错: %s" % self.last_error, flush=True)


# ══════════════════════════════════════════════════════════════════
# 把"框在哪"翻译成"按什么键、按多久"
# ══════════════════════════════════════════════════════════════════
def make_plan(d, shape):
    """返回 {lines:[...], bursts:[(key, seconds), ...]}"""
    H, W = shape[0], shape[1]
    lines, bursts = [], []
    if d.get("state") != "ok" or not d.get("center"):
        return {"lines": ["没有可用建议"], "bursts": []}
    cx, cy = d["center"][0] * W, d["center"][1] * H

    # 横向：ψ>0 表示目标在画面右侧 → 机头要往**右**转 → 按 e（e = 负偏航 = 顺时针）
    #   文豪的手动段：q→yr=+TELEOP_YAW_RATE（逆时针/左转），e→−（顺时针/右转）
    f = 1626.6183 * (W / 1920.0)
    psi = math.degrees(math.atan2(cx - W / 2.0, f))
    psi = max(-MAX_YAW_DEG, min(MAX_YAW_DEG, psi))
    if abs(psi) >= 2.0:
        k = "e" if psi > 0 else "q"          # 右 → e；左 → q
        t = abs(math.radians(psi)) / TELEOP_YAW_RATE
        bursts.append((k, t))
        lines.append("横向：目标偏 %.0f° → 按 %s %.2f s（机头转%s）" % (
            abs(psi), k.upper(), t, "右" if k == "e" else "左"))
    else:
        lines.append("横向：已居中（%.1f°）" % psi)

    # 纵向：用高度差
    dz = 0.0
    if d.get("height_m") is not None:
        dz = float(d["height_m"]) - 1.5      # 目标高度 - 当前相机高度（起飞 1.5 m）
        dz = max(-MAX_DZ_M, min(MAX_DZ_M, dz))
        if abs(dz) >= 0.3:
            k = "r" if dz > 0 else "f"
            t = abs(dz) / TELEOP_Z_SPEED
            bursts.append((k, t))
            lines.append("纵向：高度差 %+.1f m → 按 %s %.2f s" % (dz, k.upper(), t))
        else:
            lines.append("纵向：高度已够（差 %+.1f m）" % dz)
    else:
        lines.append("纵向：无高度信息，跳过")

    # 前进：按距离给步长
    dist = d.get("dist_m")
    if dist:
        step = max(2.0, min(MAX_FWD_M, 0.35 * float(dist)))
        t = step / TELEOP_XY_SPEED
        bursts.append(("w", t))
        lines.append("前进：距离 %s m → 走 %.1f m（按 W %.2f s）" % (dist, step, t))
    else:
        bursts.append(("w", 4.0 / TELEOP_XY_SPEED))
        lines.append("前进：无距离，按保守 4 m（按 W %.2f s）" % (4.0 / TELEOP_XY_SPEED))

    total = sum(b[1] for b in bursts) + DEADMAN * len(bursts)
    lines.append("合计约 %.1f s（含每段 %.1f s 死区）" % (total, DEADMAN))
    return {"lines": lines, "bursts": bursts, "total_s": round(total, 2)}


# ══════════════════════════════════════════════════════════════════
# SSH 会话
# ══════════════════════════════════════════════════════════════════
class Flight(object):
    def __init__(self, live=True, remote_cmd=None):
        self.live = live
        self.remote_cmd = remote_cmd or (
            "source /opt/ros/noetic/setup.bash; "
            "source ~/ros_ws/devel/setup.bash; bash ~/fly.sh")
        self.p = None
        self._stop = False
        # ── 阶段跟踪（只根据远端输出判断，绝不猜）──────────────────
        #   pre     : 还没进手动段（fly.sh 的 yes 确认、preflight 的 c、arm 的重试都在这里）
        #   manual  : 手动段 —— **只有这个阶段才允许注入按键**
        #   commit  : 冲刺确认段 —— Y 要透传给他
        #   auto    : Detect / Approach / Dash / Land 等自动段
        self.stage = "pre"
        self.manual_ok = False
        self.commit_ok = False

    def start(self):
        if not self.live:
            return
        cmd = ["ssh", "-tt", "-i", SSH_KEY, "-o", "StrictHostKeyChecking=no", DRONE,
               self.remote_cmd]
        env = dict(os.environ)
        for k in ("TMUX", "STY", "TERM_PROGRAM", "VSCODE_IPC_HOOK_CLI"):
            env.pop(k, None)
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, env=env, bufsize=1,
                                  text=True, encoding="utf-8", errors="replace")
        threading.Thread(target=self._reader, daemon=True).start()

    def _scan(self, s):
        """从远端输出里认阶段。只认明确的标志，认不出就不改。

        2026-11-24 教训：必须在**[gate]** 行里认，否则 bash 报错把命令原文回显出来时
        （里面正好含"段 manual"）会造成误判。真实的状态行都带 [gate]。"""
        if "[gate]" not in s:
            return
        old = self.stage
        if ("段 manual" in s) or ("── 手动段" in s) or ("★ 手动段:" in s):
            self.stage, self.manual_ok, self.commit_ok = "manual", True, False
        elif ("按 **Y** 冲刺" in s) or ("按 G 冲刺" in s) or ("按 G " in s and "冲刺" in s):
            self.stage, self.manual_ok, self.commit_ok = "commit", False, True
        elif ("★ T →" in s) or ("段 wait" in s) or ("段 align" in s) or \
                ("段 dash" in s) or ("段 brake" in s) or ("段 hold" in s) or \
                ("→ 冲刺" in s) or ("刹停" in s) or ("自动降落" in s) or ("AUTO.LAND" in s):
            self.stage, self.manual_ok, self.commit_ok = "auto", False, False
        if self.stage != old:
            print("▌ [阶段] %s → %s" % (old, self.stage), flush=True)

    def _reader(self):
        for line in self.p.stdout:
            if self._stop:
                return
            s = line.rstrip("\r\n")
            # 过滤 pty 回显：整行只有同一个键字符
            if len(s) > 8 and len(set(s)) == 1 and s[0] in "wasdrfqetlgmyx 12345":
                continue
            self._scan(s)
            print(s, flush=True)

    def send(self, ch):
        if self.p and self.p.poll() is None:
            try:
                self.p.stdin.write(ch)
                self.p.stdin.flush()
            except Exception as e:
                print("[console] 注入失败: %s" % e, flush=True)

    def burst(self, key, seconds, hz=HZ, keypoll=None):
        """按住 key 共 seconds 秒（发 hz 次/秒），再停 DEADMAN 让它归零。

        ★ 2026-11-24 安全修正：注入期间**必须**还能接管。
          以前 burst 是死循环，最长会挡住主循环 7 秒 —— 那 7 秒里按 X / L 都没用。
          现在每 1/hz 秒查一次键盘：**任何键立刻中断注入**，把那个键交回去。
        返回 ("done", None) 或 ("key", 那个键)。
        """
        n = max(1, int(round(seconds * hz)))
        step = 1.0 / hz
        for _ in range(n):
            if keypoll is not None:
                k = keypoll()
                if k:
                    time.sleep(DEADMAN)          # 先松手（死区让飞机停住）
                    return "key", k
            self.send(key)
            time.sleep(step)
        time.sleep(DEADMAN)
        return "done", None

    def close(self):
        self._stop = True
        try:
            if self.p and self.p.poll() is None:
                self.p.terminate()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════
# 主循环
# ══════════════════════════════════════════════════════════════════
def grab_frame(live, offline_frames, idx):
    if not live:
        k, f = offline_frames[idx]
        return f["clean"], "帧%d %.1fm" % (k, f.get("_z", 0))
    import urllib.request
    with urllib.request.urlopen(SNAP_RAW, timeout=5) as r:
        raw = r.read()
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    return img, "8091/raw"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true", help="连飞机（否则桌面演练）")
    ap.add_argument("--allow-move", action="store_true",
                    help="真的往飞机注入按键（默认只打印 dry-run；必须显式开）")
    ap.add_argument("--frame", type=int, default=254, help="桌面演练的起始帧号")
    ap.add_argument("--remote-cmd", default=None,
                    help="测试用：覆盖远端命令（默认 source…; bash ~/fly.sh）。"
                         "冒烟测试填 'cat' 就能看到按键原样回来")
    a = ap.parse_args()
    live = bool(a.live)                      # 连不连飞机
    moving = bool(a.allow_move) and live     # 会不会真的动

    print("=" * 70)
    print("  飞行控制台   模式：%s   注入：%s" % (
        "连飞机" if live else "桌面演练",
        "**会真的动**" if moving else "只打印(dry-run)"))
    print("=" * 70)
    print("  I = 问一次窗户（不动飞机）      U = 执行一步（一步一停）")
    print("  Esc = 退出控制台")
    print("  其余键全部原样转发：WASD / R F / Q E / 空格 / T / G / M / L / X / 1-5")
    print("  （Y / Enter / C 也全部透传 —— fly.sh、preflight、arm 的确认要它们）")
    print()

    offline = []
    idx = 0
    if not live:
        import bench_lat_crop as L
        fr = L.load_frames()
        for k in sorted(fr, key=lambda q: fr[q]["d"])[:6]:      # 从最远开始
            f = dict(fr[k])
            f["_z"] = L.FX * L.DOT_D / f["d"]
            offline.append((k, f))
        idx = max(0, min(len(offline) - 1,
                         next((i for i, (k, _) in enumerate(offline) if k == a.frame), 0)))

    print("加载三路融合定位器…（首次 20~45 s：要 import transformers + 加载两个模型）", flush=True)
    t0 = time.time()
    wf = WindowFinder()

    def _spin():
        while wf._warm.is_alive():
            time.sleep(5.0)
            print("    …还在加载（已 %.0f s）" % (time.time() - t0), flush=True)

    threading.Thread(target=_spin, daemon=True).start()
    ok = wf.wait_ready(300)
    print("预热 %s（%.1f s）\n" % ("完成" if ok else "失败", time.time() - t0), flush=True)
    adv = Advisor(wf)

    flight = Flight(live, remote_cmd=a.remote_cmd)
    flight.start()
    keys = Keys()

    try:
        while True:
            k = keys.get()
            if k is None:
                time.sleep(0.01)
                continue
            # ── Esc：退出控制台（不用 Q，Q 在手动段是左转）──────────────
            if k == "\x1b":
                print("\n▌ 退出控制台（飞机保持当前状态，不会降落）")
                break
            # ── I：问一次（任何阶段都安全；取图+推理都在后台线程，不卡键盘）──────
            if k == "i":
                if adv.busy:
                    print("\n▌ 上一问还没回来，等它", flush=True)
                    continue
                adv.ask_async(lambda: grab_frame(live, offline, idx))
                continue
            # ── U：执行一步（只在手动段允许）────────────────────────────
            if k == "u":
                if not adv.advice or adv.advice.get("state") != "ok":
                    print("\n▌ 还没有可用建议 —— 先按 I", flush=True)
                    continue
                if live and not flight.manual_ok:
                    print("\n▌ 现在不是手动段（阶段=%s），不执行。"
                          "等你看到「段 manual」再按 U" % flight.stage, flush=True)
                    continue
                plan = adv.advice.get("_plan") or {"lines": ["无计划"], "bursts": []}
                print("\n▌ 执行计划：", flush=True)
                for ln in plan["lines"]:
                    print("▌    " + ln, flush=True)
                if not plan["bursts"]:
                    print("▌    （没有要执行的动作）", flush=True)
                    continue
                if not moving:
                    print("▌    → dry-run：没有真的发按键。要真动请加 --allow-move", flush=True)
                    continue
                aborted = None
                for key, sec in plan["bursts"]:
                    print("▌    注入 %s %.2f s" % (key.upper(), sec), flush=True)
                    st, kk = flight.burst(key, sec, keypoll=keys.get)
                    if st == "key":
                        print("▌    ★ 注入被 %r 打断 —— 已松手（飞机 0.3 s 内停住）" % kk, flush=True)
                        aborted = kk
                        break
                if aborted is None:
                    print("▌    执行完毕", flush=True)
                    adv.advice = None
                else:
                    # 把打断注入的那个键照常处理掉（Esc 退出 / X 停桨 / L 降落 / 其它透传）
                    if aborted == "\x1b":
                        print("\n▌ 退出控制台（飞机保持当前状态，不会降落）")
                        break
                    adv.advice = None
                    if live and aborted and aborted != "?":
                        flight.send(aborted)
                        print("▌    已转发 %r" % aborted, flush=True)
                continue
            # ── N：桌面演练换帧 ────────────────────────────────────────
            if k == "n" and not live:
                idx = (idx + 1) % len(offline)
                print("▌ 换到 帧%d" % offline[idx][0], flush=True)
                continue
            # ── 其余键：**一律透传**（含 Y / Enter / C / Q / 数字）────────
            if live and k and k != "?":
                flight.send(k)
    except KeyboardInterrupt:
        pass
    finally:
        flight.close()
        wf.close()
        print("已退出")


if __name__ == "__main__":
    # 控制台程序：**跟随当前控制台编码**（chcp 936 → gbk），只把编不出的字符替换掉。
    #   不要强行 utf-8 —— 那会让 cmd 窗口里的中文变乱码。
    try:
        sys.stdout.reconfigure(errors="replace")
        sys.stderr.reconfigure(errors="replace")
    except Exception:
        pass
    main()
