#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_vision.py —— 起视觉链路 (新相机直连版桥 vision_bridge.py), 取代 run_vision.sh
===============================================================================
模式必须写明 (以前漏写会悄悄变成 aruco, 飞行脚本起飞前检查才拦得住):

  python3 run_vision.py gate_gcs                    # ★穿环推荐: 地面检测 (gcs_gate.py)
  python3 run_vision.py gate                        # 穿环: 机上检测
  python3 run_vision.py aruco                       # ArUco 链路 (原停靠跟踪用; stage11 已删除, 桥的 aruco 模式原样保留)

常用选项
  --gcs 192.168.2.179     地面站 IP (默认见 DEFAULT_GCS)
  --fps 15 / --tx-hz 10   相机帧率 / 上行帧率 (不给 = 桥的默认值)
  --drive                 gate: 识别结果标 REAL, 能驱动飞机 (= _gate_sim:=false)。不给 = SIM, 只影子模式能用
  --lock                  gate_gcs: 飞机侧强制 SIM, 地面 --mode 2 也不驱动 (= _gate_sim:=true)
  --pitch/--roll/--yaw/--cam-x/--cam-y/--cam-z   识别相机外参, 默认 = 下面 CAM (标定值, 只在 gate 模式传)
  --log /tmp/bridge.log   日志 (终端照常显示, 同时写这里)
  --dry                   只打印将要执行的命令
  其余 `_名字:=值` 原样传给 vision_bridge.py (和以前一样), 例: _raw_max_kb:=0

放在 tmux 新窗口里跑, 停就 Ctrl+C。老相机 (VINS) 不受影响。
===============================================================================
"""
import argparse
import os
import re
import shlex
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_GCS = "192.168.2.67"
ROS_SETUP = ["/opt/ros/noetic/setup.bash", "~/ros_ws/devel/setup.bash"]   # 和 run_vision.sh 一样

# 识别相机外参 (左目相对机体中心 = 飞控, FLU: x 前 y 左 z 上; pitch 正 = 镜头往下低)
#   ★2026-10-20 stage13_cam_calib 手持标定 (33 工位, 2.9~5.3m, 留一验证 竖直 5.1 / 横向 2.5cm):
#     pitch 5.23±0.25° (旧 1.3° 少了 ~4° → 环越远估得越高)  roll 3.40±0.31°  yaw -0.73±0.36°
#   杠杆臂卷尺: 飞控离地 16cm, 识别相机离地 4.5cm → z = -0.115; 镜头在飞控前 8cm;
#     左目在中轴线左边半个基线 3cm。(旧 z=-0.07 是按 VINS 相机/机身半高估的, 不对)
#   这里的值会以 _cam_*:= 传给桥, 覆盖 vision_bridge.py 里的默认值 —— 两边保持一致。
#   ★2026-11-05 相机重装 (基本不下垂了) → 上面 5.23/3.40/-0.73 作废。现用值 (工作参数, 待飞行核对):
#     pitch -1.1 (±0.3)  roll 0.9 (±0.3): 红点截图 + 两次 ArUco 手眼 (tools/ext_calib.py, 码 160mm) 一致
#     yaw 0.5 (±1.5): ArUco 站立组 -1.0 / 红点组 +2.0 取中 —— 室内距离不够定不准;
#       第一次真飞时看"飞近目标时横向估计随距离线性漂"的斜率修正。杠杆臂没变。
CAM = {"cam_pitch_deg": -1.1, "cam_roll_deg": 0.9, "cam_yaw_deg": 0.5,
       "cam_x": 0.080, "cam_y": 0.030, "cam_z": -0.115}

# 2026-10-30: 检测库搬进 vb09/ 包 (gate_detector.py / ring_detector.py 只剩兼容壳, 桥不再用它们)
_VB09_COMMON = ["vb09/__init__.py", "vb09/common/__init__.py", "vb09/common/geom.py", "vb09/common/wire.py"]
# 2026-11-01 接线: 插件按 registry 延迟加载; 下面是环 (--target ring) 要的文件
_VB09_FUSE = ["vb09/perception/__init__.py", "vb09/perception/registry.py", "vb09/perception/ring/__init__.py",
              "vb09/perception/ring/config.py", "vb09/perception/ring/frame.py", "vb09/perception/ring/wire.py",
              "vb09/perception/ring/geom3d.py", "vb09/perception/ring/fusion.py", "vb09/perception/ring/locate.py",
              "vb09/perception/ring/plugin.py"]
NEED = {"gate": ["vision_bridge.py", "stereo_calib.yaml", "vb09/perception/stereo_rig.py",
                 "vb09/perception/ring/det2d.py", "vb09/perception/ring/detect.py",
                 "vb09/perception/ring/draw.py"] + _VB09_COMMON + _VB09_FUSE,
        "gate_gcs": ["vision_bridge.py"] + _VB09_COMMON + _VB09_FUSE,
        "aruco": ["vision_bridge.py"] + _VB09_COMMON + _VB09_FUSE}
EXPECT = {
    "gate": ["[cam] 已打开 ...  →  [cam] 分辨率确认 3840x1080",
             "[gate] 标定 ... 工作分辨率 960x540/目   [gate] T_BC: ... frame_id=odom%s",
             "之后每 2s: [gate] xx.xHz xxms | 阶段 ... | 位姿 ok"],
    "gate_gcs": ["地面先起: python gcs_gate.py --drone <飞机IP> --mode 1   (--mode 2 才驱动飞机)",
                 "[cam] 分辨率确认 3840x1080  →  [gate_gcs] 地面标定: f=... B=60mm ...",
                 "之后每 2s: 上行 15.0Hz ...KB/张 | 回包 15.0Hz | [gate_gcs 地面检测] ... 位姿 ok"],
    "aruco": ["[cam] 已打开 /dev/v4l/by-id/...STYT...  →  [cam] 分辨率确认 1920x1080",
              "之后每 2s 一行: 相机 ✓ 10.0fps | 上行 10.0Hz 原样100% ..."],
}
ROS_ARG = re.compile(r"^_[A-Za-z]\w*:=.+$")


def stop_py(name):
    """按 /proc/<pid>/comm 判断再杀 (pgrep -f 会匹配到别的东西, 比如本程序自己)。"""
    try:
        pids = subprocess.check_output(["pgrep", "-f", name], text=True).split()
    except subprocess.CalledProcessError:
        return []
    killed = []
    for p in pids:
        if int(p) == os.getpid():
            continue
        try:
            with open("/proc/%s/comm" % p) as f:
                if f.read().strip() != "python3":
                    continue
            os.kill(int(p), signal.SIGTERM)
            killed.append(int(p))
            print("  停掉 %s (pid=%s)" % (name, p))
        except (OSError, ValueError):
            pass
    return killed


def wait_gone(pids, timeout=3.0):
    t_end = time.time() + timeout
    while time.time() < t_end and any(os.path.exists("/proc/%d" % p) for p in pids):
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser(
        description="起视觉链路 (vision_bridge.py)", formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例: python3 run_vision.py gate_gcs --gcs 192.168.2.179\n"
               "    python3 run_vision.py gate --drive --fps 15\n"
               "    python3 run_vision.py aruco _raw_max_kb:=0")
    ap.add_argument("mode", choices=("gate_gcs", "gate", "aruco"),
                    help="gate_gcs = 穿环地面检测(推荐)  gate = 穿环机上检测  aruco = 停靠跟踪")
    ap.add_argument("--gcs", default=DEFAULT_GCS, help="地面站 IP (默认 %s)" % DEFAULT_GCS)
    ap.add_argument("--target", default="ring", help="gate / gate_gcs: 目标类型 (_target, 见 vb09/perception/registry.py), "
                                                     "默认 ring。地面 gcs_gate.py --target 要一样")
    ap.add_argument("--fps", type=float, help="相机采集帧率 (_cam_fps)")
    ap.add_argument("--tx-hz", type=float, help="上行帧率上限 (_tx_hz)")
    ap.add_argument("--drive", action="store_true", help="gate 模式: 识别结果能驱动飞机 (_gate_sim:=false)")
    ap.add_argument("--lock", action="store_true", help="gate_gcs 模式: 飞机侧强制 SIM (_gate_sim:=true)")
    ap.add_argument("--pitch", type=float, default=CAM["cam_pitch_deg"], help="外参: 镜头下俯角 °")
    ap.add_argument("--roll", type=float, default=CAM["cam_roll_deg"], help="外参: 横滚 ° (右滚为正)")
    ap.add_argument("--yaw", type=float, default=CAM["cam_yaw_deg"], help="外参: 偏航 ° (左偏为正)")
    ap.add_argument("--cam-x", type=float, default=CAM["cam_x"], help="外参: 左目在机体前方 m")
    ap.add_argument("--cam-y", type=float, default=CAM["cam_y"], help="外参: 左目在机体左方 m")
    ap.add_argument("--cam-z", type=float, default=CAM["cam_z"], help="外参: 左目在机体上方 m")
    ap.add_argument("--log", default="/tmp/bridge.log", help="日志文件 (默认 /tmp/bridge.log)")
    ap.add_argument("--dry", action="store_true", help="只打印命令, 不启动")
    args, rest = ap.parse_known_args()

    bad = [a for a in rest if not ROS_ARG.match(a)]
    if bad:
        ap.error("不认识的参数 %s —— 透传给桥的参数要写成 _名字:=值" % " ".join(bad))
    if args.drive and args.mode != "gate":
        ap.error("--drive 只用于 gate 模式 (gate_gcs 由地面 --mode 2 决定是否驱动)")
    if args.lock and args.mode != "gate_gcs":
        ap.error("--lock 只用于 gate_gcs 模式")
    ros_given = {a.split(":=", 1)[0] for a in rest}

    ros = ["_mode:=%s" % args.mode, "_gcs_host:=%s" % args.gcs]
    if args.fps:
        ros.append("_cam_fps:=%g" % args.fps)
    if args.tx_hz:
        ros.append("_tx_hz:=%g" % args.tx_hz)
    if args.mode in ("gate", "gate_gcs") and "_target" not in ros_given and args.target != "ring":
        ros.append("_target:=%s" % args.target)          # ring 是桥的默认值, 不传 (命令行和以前一样)
    if args.mode in ("gate", "gate_gcs"):
        for k, v in (("_cam_pitch_deg", args.pitch), ("_cam_roll_deg", args.roll),
                     ("_cam_yaw_deg", args.yaw), ("_cam_x", args.cam_x),
                     ("_cam_y", args.cam_y), ("_cam_z", args.cam_z)):
            if k not in ros_given:
                ros.append("%s:=%g" % (k, v))
    if args.drive:
        ros.append("_gate_sim:=false")
    if args.lock:
        ros.append("_gate_sim:=true")
    ros += rest

    title = {"gate_gcs": "gate_gcs (穿环, 地面检测)", "gate": "gate (穿环, 机上检测)",
             "aruco": "aruco (停靠跟踪)"}[args.mode]
    print("=" * 70)
    print("  视觉链路   模式 ★ %s ★   地面站 %s" % (title, args.gcs))
    if args.mode in ("gate", "gate_gcs"):
        print("  目标类型 %s" % args.target)
        print("  识别相机外参 pitch=%.2f° roll=%.2f° yaw=%.2f°  x=%.3f y=%.3f z=%.3f m"
              % (args.pitch, args.roll, args.yaw, args.cam_x, args.cam_y, args.cam_z))
    if args.mode == "gate":
        print("  识别结果: %s" % ("★ REAL —— 会驱动 stage14 冲刺" if args.drive
                                 else "SIM —— stage14 只在影子模式下用 (要真飞加 --drive)"))
    elif args.mode == "gate_gcs":
        print("  识别结果: %s" % ("飞机侧强制 SIM (--lock), 地面 --mode 2 也不驱动" if args.lock
                                 else "跟随地面: gcs_gate.py --mode 2 = REAL, --mode 1 = SIM"))
    print("=" * 70)

    missing = [f for f in NEED[args.mode] if not os.path.exists(os.path.join(HERE, f))]
    if args.mode in ("gate", "gate_gcs") and args.target != "ring":
        pdir = os.path.join("vb09", "perception", args.target)      # 2026-11-05: 别的目标的插件目录也要在
        if not os.path.isfile(os.path.join(HERE, pdir, "plugin.py")):
            missing.append(pdir + "/")
    if missing:
        sys.exit("  ✗ %s 模式需要同目录的 %s" % (args.mode, ", ".join(missing)))
    if args.mode != "aruco" and not os.path.exists(os.path.join(HERE, "gate_params.json")):
        print("  ! 没有 gate_params.json, 用默认参数")

    src = "; ".join("source %s 2>/dev/null" % os.path.expanduser(s) for s in ROS_SETUP)
    py = "exec python3 -u %s %s" % (shlex.quote(os.path.join(HERE, "vision_bridge.py")),
                                    " ".join(shlex.quote(a) for a in ros))
    cmd = ["bash", "-c", "%s; %s" % (src, py)]
    if args.dry:
        print(py.replace("exec ", ""))
        return 0

    # 先停看门狗 (否则它会把抓帧节点又拉起来抢相机), 再停抓帧节点, 再停旧桥
    killed = []
    for n in ("det_cam_watchdog", "det_cam_node", "vision_bridge.py"):
        killed += stop_py(n)
    wait_gone(killed)

    print("  期望看到:")
    for ln in EXPECT[args.mode]:
        print("    " + (ln % ("" if args.drive else "_sim") if "%s" in ln else ln))
    print("  日志同时写 %s。停: Ctrl+C\n" % args.log)

    log = open(args.log, "w", encoding="utf-8", errors="replace")
    log.write("# run_vision.py %s\n# %s\n" % (" ".join(sys.argv[1:]), py))
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        for raw in p.stdout:
            line = raw.decode("utf-8", "replace")
            sys.stdout.write(line)
            sys.stdout.flush()
            log.write(line)
            log.flush()
    except KeyboardInterrupt:
        pass                                    # Ctrl+C 也发给了桥 (同一个前台进程组), 等它自己退
    finally:
        try:
            rc = p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.terminate()
            rc = p.wait()
        log.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
