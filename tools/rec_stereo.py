#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/rec_stereo.py —— 机上给识别双目截图 (红点识别的离线数据)
===============================================================================
直接开新双目 (3840×1080 MJPG), 把相机给的 **原始 JPEG 字节** 原样存成一张张 .jpg
(不解码、不缩图、不重编码 = 和桥拿到的完全一样), 每张记拍摄时刻 + 插值位姿。

默认是截图模式: 终端里回车一下存一张; 打一行字再回车 = 存一张并把这行字当标签。

    # 飞机上 (桥 run_vision.py 先停掉 —— 相机同一时间只能一个进程开; 本工具检测到桥在跑会拒绝启动)
    python3 tools/rec_stereo.py --name p1_indoor                 # 回车截图, Ctrl+C 停
    python3 tools/rec_stereo.py --burst 3                        # 每次回车存连续 3 帧 (看左右同步/抖动)
    python3 tools/rec_stereo.py --interval 2 --max-n 60          # 每 2 s 自动存一张, 存满 60 张停
    python3 tools/rec_stereo.py --ctrl auto_exposure=1 --ctrl exposure_time_absolute=40   # 固定曝光
    python3 tools/rec_stereo.py --list-ctrls                     # 只打印相机支持的 v4l2 控件, 不录
    python3 tools/rec_stereo.py --name extcal --burst 3 --aruco 30 --marker-mm 160   # 外参标定: 实时提示码的位置

    ★ 看画面: 手机 / 笔记本连同一个 WiFi, 浏览器开 http://<飞机IP>:8092/   (启动时会打印地址)
      画面 = 左目 + 右下角小图右目; 3×3 格子里存过的格子变绿 (数字 = 存了几张);
      --aruco 时: 码绿框 = 左右目都看到, 黄框 = 只有左目; 顶上一行 L/R 是否看到、在哪一格、距离、STILL/MOVING。
      终端里每存一张也会报告 "码: 左✓ 右✓ 左上 1.3 m 静止", 没看到码会提示重拍。

    一张约几百 KB (3840×1080 原始 MJPG), 100 张几十 MB。录完目录打个 zip 就能发。
    VINS / mavros 没开也能截 (没有位姿, 时间用本机时钟)。

输出目录 (~/rec/<YYYYmmdd_HHMMSS>[_name]/):
    NNNN_<标签>.jpg  每张 = 左目|右目 并排, 原始 JPEG
    frames.csv    n, file, t_cap (ROS 秒, 和桥的 t_cap 同一时钟), label, x y z qx qy qz qw (按 t_cap 插值, 没有就空)
    poses.csv     /mavros/local_position/pose 原样 (只留截图前后各 2 s, 不是全程)
    meta.json     分辨率、设备、v4l2 控件实际值、相机外参 (run_vision.CAM)、代码版本
    stereo_calib.yaml   截图时用的标定 (拷一份, 以后重标了也知道用的是哪份)

读回 / 导出校正图: python3 tools/rec_read.py <目录>
Python 3.8。
"""
import argparse
import csv
import glob
import hashlib
import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OLD_CAM_MARK = "SPCA2100"            # 老相机 (VINS) 的 by-id 名字, 绝不打开 (同 vision_bridge)
POSE_TOPIC = "/mavros/local_position/pose"
BUSY = ("vision_bridge.py", "det_cam_node.py", "det_cam_watchdog.py")
MIN_FREE_MB = 1024                   # 磁盘剩这么多就停


def find_device():
    """同 vision_bridge.find_device。"""
    for pat in ("*STYT*video-index0", "*USB_Camera*video-index0", "*STYT*", "*USB_Camera*"):
        hits = [p for p in sorted(glob.glob("/dev/v4l/by-id/" + pat)) if OLD_CAM_MARK not in p]
        if hits:
            return hits[0]
    return ""


def busy_procs():
    out = []
    for name in BUSY:
        try:
            pids = subprocess.check_output(["pgrep", "-f", name], universal_newlines=True).split()
        except Exception:
            pids = []
        pids = [p for p in pids if int(p) != os.getpid()]
        if pids:
            out.append("%s (pid %s)" % (name, ",".join(pids)))
    return out


def v4l2(dev, *args):
    try:
        return subprocess.check_output(["v4l2-ctl", "-d", dev] + list(args),
                                       stderr=subprocess.STDOUT, universal_newlines=True)
    except FileNotFoundError:
        return "(没有 v4l2-ctl: sudo apt install v4l-utils)"
    except subprocess.CalledProcessError as e:
        return "(v4l2-ctl 失败: %s)" % e.output.strip()


def raw_jpeg(buf):
    """retrieve() 结果 → 原始 JPEG bytes (同 vision_bridge.as_jpeg)。驱动给的是解码图就返回 None。"""
    if buf is None:
        return None
    a = np.asarray(buf)
    if a.ndim == 3 or (a.ndim == 2 and a.shape[0] > 1 and a.shape[1] > 1):
        return None
    b = a.reshape(-1).tobytes()
    if len(b) < 4 or b[:2] != b"\xff\xd8":
        return None
    end = b.rfind(b"\xff\xd9")
    return b[:end + 2] if end > 0 else b


def sha12(path):
    try:
        with open(path, "rb") as fh:
            return hashlib.sha256(fh.read()).hexdigest()[:12]
    except Exception:
        return ""


def git_rev():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                       stderr=subprocess.DEVNULL, universal_newlines=True).strip()
    except Exception:
        return ""


def cam_extrinsics():
    """run_vision.CAM (左目外参), 记进 meta。读不到就算了。"""
    try:
        sys.path.insert(0, ROOT)
        import run_vision
        return dict(run_vision.CAM)
    except Exception:
        return None


class Clock(object):
    """有 ROS 就用 rospy.Time.now() (= 桥的 t_cap 时钟, 和 VINS / mavros 位姿对得上), 没有就 time.time()。"""

    def __init__(self, use_ros):
        self.rospy = None
        self.poses = []
        self.lock = threading.Lock()
        if not use_ros:
            return
        try:
            import rospy
            from geometry_msgs.msg import PoseStamped
            import rosgraph
            if not rosgraph.is_master_online():
                print("[rec] ROS master 没开 —— 不记位姿, 时间用本机时钟")
                return
            rospy.init_node("rec_stereo", anonymous=True, disable_signals=True)
            rospy.Subscriber(POSE_TOPIC, PoseStamped, self._on_pose, queue_size=50)
            self.rospy = rospy
        except Exception as e:
            print("[rec] 没有 rospy (%s) —— 不记位姿, 时间用本机时钟" % e)

    def _on_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.poses.append((m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def now(self):
        return self.rospy.Time.now().to_sec() if self.rospy else time.time()

    def take_poses(self):
        with self.lock:
            out, self.poses = self.poses, []
        return out



POSE_KEEP_S = 2.0                    # poses.csv 只留每张截图前后这么久


def safe_label(text):
    keep = []
    for ch in text.strip():
        keep.append(ch if (ch.isalnum() or ch in "-_.") else "_")
    return "".join(keep)[:40].strip("_")


def stdin_lines(q, stop):
    """终端里每一行 (含空行) = 一次截图请求。"""
    while not stop.is_set():
        line = sys.stdin.readline()
        if not line:
            return
        q.put(line.strip())


def interp_pose(poses, t, max_gap=0.1):
    """同 vision_bridge.PoseBuffer.at (位置线性 + 四元数 nlerp) → [x y z qx qy qz qw] 或 None。"""
    ts = [p[0] for p in poses]
    i = int(np.searchsorted(ts, t))
    if i == 0 or i >= len(poses):
        return None
    a, b = poses[i - 1], poses[i]
    if b[0] - a[0] > max_gap:
        return None
    s = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    pa, pb = np.array(a[1:4]), np.array(b[1:4])
    qa, qb = np.array(a[4:]), np.array(b[4:])
    if qa @ qb < 0:
        qb = -qb
    q = qa + s * (qb - qa)
    q /= np.linalg.norm(q)
    return list(pa + s * (pb - pa)) + list(q)


CELL_NAMES = [["左上", "上中", "右上"], ["左中", "中间", "右中"], ["左下", "下中", "右下"]]
CELL_EN = [["TL", "T", "TR"], ["L", "C", "R"], ["BL", "B", "BR"]]
PREVIEW_HZ = 5.0
STILL_WIN_S = 0.5                    # 看最近这么久的位姿判断稳没稳
STILL_MM, STILL_DEG = 5.0, 0.3


def make_aruco(dict_name):
    """→ detect(gray) -> (corners, ids) ; 没有 cv2.aruco 返回 None。兼容 OpenCV 4.2 (飞机) 和 ≥4.7。"""
    if not hasattr(cv2, "aruco"):
        return None
    a = cv2.aruco
    d = a.getPredefinedDictionary(getattr(a, dict_name)) if hasattr(a, "getPredefinedDictionary") \
        else a.Dictionary_get(getattr(a, dict_name))
    if hasattr(a, "ArucoDetector"):
        det = a.ArucoDetector(d, a.DetectorParameters())
        return lambda g: det.detectMarkers(g)[:2]
    prm = a.DetectorParameters_create()
    return lambda g: a.detectMarkers(g, d, parameters=prm)[:2]


def motion(poses, t_now):
    """最近 STILL_WIN_S 内位姿的最大位移 (mm) 和最大转角 (°)；没有位姿 → None。"""
    if not poses:
        return None
    tail = [p for p in poses[-200:] if p[0] >= t_now - STILL_WIN_S]
    if len(tail) < 3:
        return None
    P = np.array([p[1:4] for p in tail])
    Q = np.array([p[4:8] for p in tail])
    dmm = float(np.max(np.linalg.norm(P - P[-1], axis=1))) * 1e3
    dots = np.clip(np.abs(Q @ Q[-1]) / (np.linalg.norm(Q, axis=1) * np.linalg.norm(Q[-1])), 0, 1)
    ddeg = float(np.degrees(2 * np.arccos(dots.min())))
    return dmm, ddeg


class Monitor(object):
    """
    后台线程: 解码 (缩一半) + 找码 + 画预览 + 统计 3×3 覆盖。采集循环只把 JPEG 丢进来, 不被拖慢。
      show(jpg, mot)      最新一帧 (只留最新)
      check(fn, jpg, mot) 刚存的一张: 终端报告码看没看到、在哪一格
    """

    def __init__(self, aruco_id, aruco_dict, marker_mm, fx_full, preview, eye_w=1920):
        self.id = aruco_id
        self.detect = make_aruco(aruco_dict) if aruco_id is not None else None
        if aruco_id is not None and self.detect is None:
            print("[rec] ★ 这台机器的 OpenCV 没有 aruco 模块 —— 不提示码的位置, 只给画面 (离线照样能标)")
        self.marker_m = marker_mm / 1000.0
        self.fx = fx_full
        self.eye_w = int(eye_w) or 1920
        self.pv = preview
        self.cover = np.zeros((3, 3), int)
        self.n_saved = 0
        self.lock = threading.Lock()
        self.latest = None
        self.saved = queue.Queue()
        self.ev = threading.Event()
        self.stop = threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def show(self, jpg, mot):
        with self.lock:
            self.latest = (jpg, mot)
        self.ev.set()

    def check(self, fn, jpg, mot):
        self.saved.put((fn, jpg, mot))
        self.ev.set()

    def idle(self):
        with self.lock:
            return self.latest is None

    # ---- 分析一帧 (半分辨率) ----
    def _find(self, gray):
        if self.detect is None:
            return None
        corners, ids = self.detect(gray)
        if ids is None:
            return None
        for c, i in zip(corners, np.asarray(ids).ravel()):
            if int(i) == self.id:
                return c.reshape(4, 2)
        return None

    def analyse(self, jpg):
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if img is None:
            return None
        w2 = img.shape[1] // 2
        L, R = img[:, :w2], img[:, w2:]
        info = {"L": None, "R": None, "cell": None, "dist": None, "img": (L, R)}
        if self.detect is not None:
            info["L"] = self._find(cv2.cvtColor(L, cv2.COLOR_BGR2GRAY))
            info["R"] = self._find(cv2.cvtColor(R, cv2.COLOR_BGR2GRAY))
            c = info["L"]
            if c is not None:
                cx, cy = c.mean(0)
                info["cell"] = (min(2, int(3 * cy / L.shape[0])), min(2, int(3 * cx / L.shape[1])))
                scale = float(self.eye_w) / max(L.shape[1], 1)
                side = np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]) * scale  # 全分辨率像素
                info["dist"] = self.fx * self.marker_m / max(side, 1e-3)
        return info

    @staticmethod
    def mot_text(mot, zh=True):
        if mot is None:
            return "无位姿" if zh else "NO POSE"
        still = mot[0] < STILL_MM and mot[1] < STILL_DEG
        if zh:
            return ("静止" if still else "★晃动 %.0fmm/%.1f°" % mot)
        return ("STILL" if still else "MOVING %.0fmm %.1fdeg" % mot)

    def _report(self, fn, jpg, mot):
        info = self.analyse(jpg)
        self.n_saved += 1
        if info is None:
            print("    ★ 解码失败")
            return
        if self.detect is None:
            print("    %s" % self.mot_text(mot))
            return
        if info["L"] is None:
            print("    ★ 左目没看到码 —— 这张没用, 调整后重拍")
            return
        r, c = info["cell"]
        self.cover[r, c] += 1
        miss = [CELL_NAMES[i][j] for i in range(3) for j in range(3) if self.cover[i, j] == 0]
        print("    码: 左✓ 右%s  %s  %.1f m  %s   |  还没拍过的格子: %s"
              % ("✓" if info["R"] is not None else "✗(只有左目, 能用)", CELL_NAMES[r][c], info["dist"],
                 self.mot_text(mot), "、".join(miss) if miss else "无, 都拍过了"))

    def _draw(self, info, mot):
        L, R = info["img"]
        vis = L.copy()
        h, w = vis.shape[:2]
        over = vis.copy()
        for i in range(3):
            for j in range(3):
                if self.cover[i, j]:
                    cv2.rectangle(over, (j * w // 3, i * h // 3), ((j + 1) * w // 3, (i + 1) * h // 3), (0, 160, 0), -1)
        vis = cv2.addWeighted(over, 0.25, vis, 0.75, 0)
        for k in (1, 2):
            cv2.line(vis, (k * w // 3, 0), (k * w // 3, h), (200, 200, 200), 1)
            cv2.line(vis, (0, k * h // 3), (w, k * h // 3), (200, 200, 200), 1)
        for i in range(3):
            for j in range(3):
                if self.cover[i, j]:
                    cv2.putText(vis, str(self.cover[i, j]), (j * w // 3 + 8, (i + 1) * h // 3 - 10), 0, 0.9,
                                (0, 255, 0), 2)
        if info["L"] is not None:
            col = (0, 255, 0) if info["R"] is not None else (0, 220, 255)
            cv2.polylines(vis, [info["L"].astype(np.int32)], True, col, 3)
        # 下方一栏: 右目小图 + 状态
        rs = cv2.resize(R, (w // 3, h // 3))
        if info["R"] is not None:
            cv2.polylines(rs, [(info["R"] / 3).astype(np.int32)], True, (0, 255, 0), 2)
        panel = np.zeros((rs.shape[0], w, 3), np.uint8)
        panel[:, w - rs.shape[1]:] = rs
        cv2.putText(panel, "right eye ->", (w - rs.shape[1] - 150, 24), 0, 0.6, (200, 200, 200), 1)
        if self.detect is not None:
            seen = "L:%s  R:%s" % ("OK" if info["L"] is not None else "--", "OK" if info["R"] is not None else "--")
            pos = ("%s  %.1f m" % (CELL_EN[info["cell"][0]][info["cell"][1]], info["dist"])) if info["L"] is not None \
                else "marker not seen"
        else:
            seen, pos = "", ""
        still = mot is not None and mot[0] < STILL_MM and mot[1] < STILL_DEG
        rows = [(seen, (0, 255, 0) if info["L"] is not None and info["R"] is not None else (0, 200, 255)),
                (pos, (255, 255, 255)),
                (self.mot_text(mot, zh=False), (0, 255, 0) if still else (0, 200, 255)),
                ("saved %d" % self.n_saved, (255, 255, 255))]
        for k, (t, c) in enumerate(rows):
            if t:
                cv2.putText(panel, t, (10, 32 + 40 * k), 0, 0.9, c, 2)
        vis = np.vstack([vis, panel])
        meta = {"aruco_L": info["L"] is not None, "aruco_R": info["R"] is not None,
                "cell": CELL_NAMES[info["cell"][0]][info["cell"][1]] if info["cell"] else None,
                "dist_m": info["dist"], "motion_mm_deg": list(mot) if mot else None, "saved": self.n_saved,
                "cover": self.cover.tolist()}
        return vis, meta

    def _run(self):
        while not self.stop.is_set():
            if not self.ev.wait(0.5):
                continue
            self.ev.clear()
            while not self.saved.empty():                 # 存下的先报告 (覆盖统计要先更新)
                self._report(*self.saved.get())
            with self.lock:
                item, self.latest = self.latest, None
            if item is None or self.pv is None:
                continue
            try:
                info = self.analyse(item[0])
                if info is not None:
                    vis, meta = self._draw(info, item[1])
                    self.pv.publish(vis, meta=meta)
            except Exception as e:
                print("[rec] 预览出错 (不影响截图): %s" % e)

    def close(self):
        while not self.saved.empty():
            self._report(*self.saved.get())
        self.stop.set()


def start_preview(port, host):
    try:
        sys.path.insert(0, ROOT)
        from vb09.common import preview_server
    except Exception as e:
        print("[rec] 没有 vb09/common/preview_server.py (%s) —— 没有网页预览, 截图照常" % e)
        return None
    pv = preview_server.start(port=port, host=host, max_width=960, quality=70)
    if pv is None:
        return None
    ips = []
    try:
        ips = subprocess.check_output(["hostname", "-I"], universal_newlines=True).split()
    except Exception:
        pass
    ips = [ip for ip in ips if ":" not in ip] or ["<飞机IP>"]
    print("[rec] 画面预览: " + "  ".join("http://%s:%d/" % (ip, port) for ip in ips)
          + "   (手机/笔记本连同一个 WiFi 用浏览器打开)")
    return preview_server


def calib_fx(path):
    """标定里的左目 fx (全分辨率), 读不到用 1566。"""
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        K = fs.getNode("K1").mat()
        fs.release()
        return float(K[0, 0]) if K is not None else 1566.0
    except Exception:
        return 1566.0


def main():
    ap = argparse.ArgumentParser(description="机上给识别双目截图 (全分辨率原始 JPEG + 位姿)")
    ap.add_argument("--out", default=os.path.expanduser("~/rec"), help="根目录, 默认 ~/rec")
    ap.add_argument("--name", default="", help="目录名后缀, 例 p1_indoor")
    ap.add_argument("--device", default="", help="不给 = 按 /dev/v4l/by-id 自动找新相机")
    ap.add_argument("--w", type=int, default=3840)
    ap.add_argument("--h", type=int, default=1080)
    ap.add_argument("--fps", type=float, default=15.0, help="请求相机帧率 (桥 gate 模式也是 15)")
    ap.add_argument("--burst", type=int, default=1, help="每次截图存连续几帧")
    ap.add_argument("--interval", type=float, default=0.0, help="每隔这么多秒自动存一张 (0 = 只在回车时存)")
    ap.add_argument("--max-n", type=int, default=300, help="存满这么多张自动停 (防止忘了关)")
    ap.add_argument("--ctrl", action="append", default=[], metavar="名字=值",
                    help="打开相机后用 v4l2-ctl 设置控件 (可多次), 例 --ctrl auto_exposure=1")
    ap.add_argument("--list-ctrls", action="store_true", help="只打印相机支持的控件和格式, 不录")
    ap.add_argument("--calib", default=os.path.join(ROOT, "stereo_calib.yaml"))
    ap.add_argument("--no-ros", action="store_true", help="不连 ROS (纯本机时钟, 不记位姿)")
    ap.add_argument("--force", action="store_true", help="桥在跑也硬开 (一般会打不开)")
    ap.add_argument("--aruco", type=int, default=None, metavar="ID",
                    help="实时找这个 ArUco 码 (外参标定用), 网页和终端提示它在哪一格、多远")
    ap.add_argument("--aruco-dict", default="DICT_4X4_50")
    ap.add_argument("--marker-mm", type=float, default=160.0, help="码黑框实际边长 (只用来显示距离)")
    ap.add_argument("--preview-port", type=int, default=8092)
    ap.add_argument("--preview-host", default="0.0.0.0", help="0.0.0.0 = 同一 WiFi 的手机能看; 127.0.0.1 = 只能 SSH 转发看")
    ap.add_argument("--no-preview", action="store_true", help="不开网页预览")
    args = ap.parse_args()

    dev = args.device or find_device()
    if not dev or not os.path.exists(dev):
        sys.exit("[rec] 找不到新相机 (/dev/v4l/by-id/*STYT*)。ls /dev/v4l/by-id/")
    if args.list_ctrls:
        print(v4l2(dev, "--list-ctrls-menus"))
        print(v4l2(dev, "--list-formats-ext"))
        return
    busy = busy_procs()
    if busy and not args.force:
        sys.exit("[rec] 相机被占用: %s —— 先停掉桥 (run_vision.py 那个窗口 Ctrl+C)" % "; ".join(busy))

    stamp = time.strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(args.out, stamp + ("_" + args.name if args.name else ""))
    os.makedirs(outdir)
    print("[rec] 输出 %s  (磁盘剩 %.1f GB)" % (outdir, shutil.disk_usage(outdir).free / 1e9))

    clock = Clock(not args.no_ros)

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit("[rec] 打不开 %s (被占用? fuser %s)" % (dev, dev))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))     # 先 FOURCC 再尺寸 (同桥)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.h)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)                                # 原始 MJPG, 不解码
    for c in args.ctrl:
        print("[rec] v4l2 %s %s" % (c, v4l2(dev, "--set-ctrl", c).strip() or "ok"))

    first = None
    for _ in range(30):
        if cap.grab():
            ok, buf = cap.retrieve()
            first = raw_jpeg(buf if ok else None)
            if first is not None:
                break
    if first is None:
        cap.release()
        sys.exit("[rec] 拿不到原始 MJPG (驱动不支持 CONVERT_RGB=0?) —— 不存, 免得存成重编码的")
    g = cv2.imdecode(np.frombuffer(first, np.uint8), cv2.IMREAD_GRAYSCALE)
    wh = (g.shape[1], g.shape[0]) if g is not None else (0, 0)
    if wh != (args.w, args.h):
        print("[rec] ★ 实际分辨率 %dx%d ≠ 请求 %dx%d —— 和标定对不上, 测距会错" % (wh + (args.w, args.h)))

    if os.path.exists(args.calib):
        shutil.copy(args.calib, os.path.join(outdir, "stereo_calib.yaml"))
    else:
        print("[rec] ★ 没找到标定 %s, 没拷" % args.calib)
    meta = {
        "start": stamp, "device": dev, "req_wh": [args.w, args.h], "actual_wh": list(wh),
        "req_fps": args.fps, "burst": args.burst, "interval": args.interval,
        "ctrl_set": args.ctrl, "ctrls": v4l2(dev, "--list-ctrls"),
        "calib": args.calib, "calib_sha12": sha12(args.calib),
        "cam_extrinsics": cam_extrinsics(), "git": git_rev(),
        "clock": "ros" if clock.rospy else "wall", "pose_topic": POSE_TOPIC if clock.rospy else None,
        "layout": "左右并排 (一张 JPEG = 左目|右目, 和桥的 gate 模式一样)",
    }
    with open(os.path.join(outdir, "meta.json"), "w") as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=1)

    pv = None if args.no_preview else start_preview(args.preview_port, args.preview_host)
    mon = Monitor(args.aruco, args.aruco_dict, args.marker_mm, calib_fx(args.calib), pv, eye_w=wh[0] // 2)
    t_pv = 0.0

    lq = queue.Queue()
    stop = threading.Event()
    threading.Thread(target=stdin_lines, args=(lq, stop), daemon=True).start()

    shots = []                 # (n, file, t, label)
    poses = []                 # 全部位姿 (内存里, 结束时只写截图附近的)
    pending = []               # 还要存的帧: label
    n_saved, nbytes = 0, 0
    t0 = t_auto = time.time()
    t_rep = t0
    fails = 0
    why = "Ctrl+C"
    if args.interval > 0:
        print("[rec] 每 %.1f s 自动存一张; 打字回车 = 立刻存一张带标签的。Ctrl+C 停" % args.interval)
    else:
        print("[rec] 回车 = 存一张 (先打字再回车 = 带标签, 例: 10cm 5m 白墙)。Ctrl+C 停")
    try:
        while True:
            if not cap.grab():
                fails += 1
                if fails >= 20:
                    why = "连续取帧失败 (USB 松了?)"
                    break
                continue
            t = clock.now()                    # 帧到手的时刻 = t_cap (同桥)
            fails = 0
            poses.extend(clock.take_poses())
            now = time.time()
            while not lq.empty():
                pending.extend([lq.get()] * max(1, args.burst))
            if args.interval > 0 and now - t_auto >= args.interval:
                t_auto = now
                pending.extend(["auto"] * max(1, args.burst))
            want_pv = pv is not None and now - t_pv >= 1.0 / PREVIEW_HZ and mon.idle()
            if not pending and want_pv:
                ok, buf = cap.retrieve()
                jpg = raw_jpeg(buf if ok else None)
                if jpg is not None:
                    mon.show(jpg, motion(poses, t))
                    t_pv = now
                continue
            if not pending:
                if now - t_rep > 30:
                    print("[rec] 相机正常, 已存 %d 张" % n_saved)
                    t_rep = now
                continue
            label = pending.pop(0)
            ok, buf = cap.retrieve()
            jpg = raw_jpeg(buf if ok else None)
            if jpg is None:
                pending.insert(0, label)
                continue
            lab = safe_label(label)
            fn = "%04d%s.jpg" % (n_saved, ("_" + lab) if lab else "")
            with open(os.path.join(outdir, fn), "wb") as fh:
                fh.write(jpg)
            shots.append((n_saved, fn, t, label))
            n_saved += 1
            nbytes += len(jpg)
            t_rep = now
            mot = motion(poses, t)
            print("[rec] 存 %s  %.0f KB  位姿 %s" % (fn, len(jpg) / 1e3, "有" if clock.rospy else "无"))
            mon.check(fn, jpg, mot)
            if want_pv:
                mon.show(jpg, mot)
                t_pv = now
            if n_saved >= args.max_n:
                why = "到 --max-n"
                break
            if shutil.disk_usage(outdir).free < MIN_FREE_MB * 1e6:
                why = "磁盘不够"
                break
    except KeyboardInterrupt:
        pass
    stop.set()
    cap.release()
    mon.close()
    time.sleep(0.15)                           # 收最后一点位姿
    poses.extend(clock.take_poses())
    poses.sort()

    with open(os.path.join(outdir, "frames.csv"), "w") as fh:
        w = csv.writer(fh)
        w.writerow(["n", "file", "t_cap", "label", "x", "y", "z", "qx", "qy", "qz", "qw"])
        n_pose = 0
        for n, fn, t, label in shots:
            p = interp_pose(poses, t) if poses else None
            n_pose += p is not None
            w.writerow([n, fn, "%.6f" % t, label] + (["%.6f" % v for v in p] if p else [""] * 7))
    with open(os.path.join(outdir, "poses.csv"), "w") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        ts = [s[2] for s in shots]
        for p in poses:
            if any(abs(p[0] - t) <= POSE_KEEP_S for t in ts):
                w.writerow(["%.6f" % p[0]] + ["%.6f" % v for v in p[1:]])
    print("[rec] 停止 (%s)。存 %d 张, %.1f MB, 有位姿 %d 张 → %s"
          % (why, n_saved, nbytes / 1e6, n_pose if shots else 0, outdir))
    print("[rec] 打包: cd %s && zip -r %s.zip %s" % (os.path.dirname(outdir), os.path.basename(outdir),
                                                    os.path.basename(outdir)))


if __name__ == "__main__":
    main()
