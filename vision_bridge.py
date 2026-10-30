#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_bridge.py — 飞机侧图传/检测桥 (新相机直连版, 2026-10-12)
===============================================================================
**桥自己打开识别相机**, 不再需要 det_cam_node / det_cam_watchdog / prep_det_cam.sh,
也没有 cam_topic / tx_width 这些会残留在 ROS master 上的参数。
老相机 (机载 SPCA2100 双目) **只给 VINS 用**, 本程序完全不碰它。

  启动:  bash run_vision.sh [地面站IP]        (同目录; 会先停掉旧的抓帧节点/看门狗/旧桥)
  或者:  python3 vision_bridge.py _gcs_host:=192.168.2.179

数据流
-------------------------------------------------------------------------------
  新相机 (USB 1bcf:0b15, /dev/v4l/by-id/*STYT*video-index0)
    │  V4L2, MJPG 1920×1080 (= 并排帧的右目, 地面站 gcs_aruco_det2 的 K2/D2 就是它)
    ▼
  采集线程: 每帧 grab() (只出队, 不解码) → 打 t_cap → 到发送间隔才 retrieve()
    │        ★ 一直在 grab ⇒ 驱动里不会囤旧帧, t_cap 就是"刚到手"的时刻
    ▼
  上行线程: 相机原始 JPEG **原样转发** (不解码不重编码: 最清楚、最省 Khadas CPU)
    │        或 (原始帧太大 / 要缩小时) 解码 → 缩放 → JPEG q=jpeg_q
    ▼
  ZMQ PUB tcp://*:5555  单帧 msgpack {seq, t_cap, w, h, K, pose, fc, ph, rtt_*, jpg}
  ZMQ SUB tcp://<gcs>:5556 ← {t_cap 原样回填, found, target(odom), sim}
    ▼
  /track/target  PointStamped (header.stamp = 拍摄时刻; sim → frame_id "odom_sim")
  /bridge/rtt    String       往返时间
  /vision_bridge/cam  String (JSON, 1Hz)  相机状态 —— flight_monitor「识别相机」读它

掉线自愈 (取代 det_cam_watchdog)
-------------------------------------------------------------------------------
  连续 3 次 grab 失败, 或 2s 没有新帧 → release → 按 by-id 重新找设备 → 重开,
  失败就每秒再试。重开次数和原因进 /vision_bridge/cam 和日志。
  V4L2 的 select 超时用环境变量压到 2s (默认 10s), 拔线后最快 ~2s 发现。

参数 (~私有参数, 只认**这次命令行**给的 _xxx:=值; ROS master 上残留的、这次没给的会被删掉
      并回到默认 —— 所以不传 = 一定是下面的默认值, 不会被上次的设置或旧起桥脚本影响)
  ~gcs_host     地面站 IP            默认 192.168.2.179
  ~device       相机设备             默认 "" = 自动按 by-id 找 (排除老相机 SPCA2100)
  ~cam_w ~cam_h 采集分辨率           默认 1920 / 1080 (地面站内参就是按这个标的, 别改)
  ~cam_fps      请求相机帧率         默认 10 (和老相机同一条 USB 2.0, 先保守)
  ~tx_hz        上行帧率上限         默认 10 (= stage11 的 F_DETECT)
  ~tx_max_w     上行最大宽度         默认 1920 (<1920 时必须重编码, 远处小标记会丢)
  ~raw_max_kb   原始帧 ≤ 这么大就原样转发, 否则重编码   默认 260 (0 = 永远重编码)
  ~jpeg_q       重编码时的 JPEG 质量  默认 85
  ~status_topic 飞行脚本阶段话题     默认 /flight_ctrl/status
  ~ground_z     目标平面高度 (旧几何用) 默认 0.0

和旧版 (2026-10-08 s11) 相比修掉的问题
-------------------------------------------------------------------------------
  1. 相机参数在 import 时读 (那时 init_node 还没跑, ~xxx 解析不到) → 现在全在 main() 里读。
  2. 参数残留在 master: 飞完新相机再飞老相机 → "视觉全程无效" → 不再用这些参数。
  3. 抓帧节点读帧失败无限 continue, 不重开设备 → 现在自己重开 (不需要看门狗)。
  4. 抓帧节点按 sleep 节流: 相机若比 10fps 快, read() 会拿到驱动里排队的旧帧,
     但时间戳打的是"现在" → t_cap 偏新, stage11 的年龄降级被骗 → 现在持续 grab。
  5. 时间戳在解码之后才打 → 现在在 grab 返回时打。
  6. MJPG → 解码成 6.2MB bgr8 → ROS 序列化 → 桥里再压一次 JPEG (两次有损 + 一堆 CPU)
     → 现在默认原样转发相机的 JPEG。
  7. camera_info 的 fy 被写成 fx (1573.80), 实际标定 fy=1578.16 → 头里 K 用真值。
  8. caminfo_cb 无锁改全局内参 → 已删除 (内参是常量)。
  9. 文档写"frame 0 头 + frame 1 JPEG 两帧", 实际早已是单帧 (CONFLATE 不能配多帧) → 已更正。

依赖: OpenCV, pyzmq, msgpack (飞机上都有)。没有 cv_bridge 依赖了。

★ 2026-10-19 (穿环首飞问题 §1 §2, gate / gate_gcs 共用)
  · /gate/status 加 cov (世界系 3×3 协方差, m², 行主序) —— stage12 用它把 σ 拆成横向 / 深度
  · /gate/status 加 ring_w / ring_cov / ring_src / ring_c / n / aim (见 _ring_extra): target 是红点,
    红点没定位上时 stage12 改用每帧的环心观测, 按环心穿
  · 环估计 gate_reset_age 秒没被新检测更新 → 丢掉旧状态, 从下一帧重新初始化
    (默认 = max(1s, gate_params 的 lost_timeout))。落地后 / VINS 跳过后不再空转出旧环。
    过程噪声 q 在 gate_detector.py 里, 这里没改

★ gate 模式 (2026-10-15 新增, _mode:=gate) —— 格子环 + 红点, **机上**识别
-------------------------------------------------------------------------------
  默认 _mode:=aruco 完全是上面的老行为, 一行没变。gate 模式下:
    相机 3840×1080 MJPG 双目并排 → 检测线程(只处理最新帧):
      MJPG 降采样解码(1920×540) → 双目校正 → gate_detector.process(L, R, t_cap, T_WC)
      T_WC = T_odom←body(t_cap, 位姿缓冲插值) · T_BC(左目外参, 参数给)
    → /track/target  PointStamped (odom 系, stamp = t_cap)  —— 默认 frame_id "odom_sim"!
                     (_gate_sim:=false 才是 "odom", 才会真的驱动 stage11; 机上检测没有地面开关, 只有这一道)
    → /gate/target   PoseWithCovarianceStamped (位置 + 3×3 协方差)
    → /gate/status   String JSON (阶段/距离/来源/耗时/环/红点...) 每帧
    上行: 发校正后左目小图(带叠加)给地面站看; 地面站回包里的 target 在 gate 模式下**不发布**。
  需要同目录: gate_detector.py  ring_detector.py  stereo_calib.yaml  gate_params.json
  例:  bash run_vision.sh 192.168.2.179 _mode:=gate _cam_fps:=15 _cam_pitch_deg:=0
  ★ stage11 现在的 STANDOFF 控制律会在 1.2~1.5m 停住, 不会去撞; 冲撞控制另做。

★ gate_gcs 模式 (2026-10-15 新增, _mode:=gate_gcs) —— 格子环 + 红点, **地面**识别
-------------------------------------------------------------------------------
  机上只做: 采集 → 1920×540 并排帧 (MJPG 降采样解码 + JPEG q=jpeg_q) → 上行;
            收地面观测 → 按 t_cap 插值位姿 → GateFuser 融合 → 发布 (同 gate 模式的三个话题)。
  地面 (gcs_gate.py) 做全部检测, 只回**相机(校正左目)系观测**, 不需要也拿不到位姿。
    上行包头: sbs=True, w/h=并排帧尺寸, gate=飞机最新融合结果 (地面只拿来显示)
    下行包:   {t_cap, found, gate: 观测, rig: {f, R1, ...}, sim}
  · 位姿在飞机上、在回包到达时插值 —— 那时 t_cap 前后的位姿早已到齐, 不用外推。
  · 估计器状态留在飞机上: 断链 / 地面重启都不丢。地面标定变了 (rig 不同) → 估计器重置。
  · 单保险 (同 aruco 链路): 地面 gcs_gate --mode 2 就驱动飞机; --mode 1 (默认) / --probe 不驱动。
    飞机 _gate_sim 在 gate_gcs 下默认 false = 跟随地面; 显式给 _gate_sim:=true 可在飞机上强制锁住。
  · 地面 --mode 0 (不检测) 的回包不带 gate 字段、带 mode=0 —— 只统计往返, 不发布, 不告警。
  · 回包里没有 gate 字段 (地面跑成了 gcs_aruco_det2) → 一律不发布, 只告警。
  需要同目录: gate_detector.py ring_detector.py (gate_params.json 可选; 标定文件在地面)
  例:  bash run_vision.sh 192.168.2.179 _mode:=gate_gcs
       地面: python gcs_gate.py --drone 192.168.2.20 --mode 1
  ★2026-10-20 外参默认值已换成 stage13_cam_calib 的标定结果 (pitch 5.23 roll 3.40 yaw -0.73,
  lever 0.080/0.030/-0.115), 启动命令里不要再带 _cam_*:= (会覆盖默认值)。下面是旧说明:
  外参 _cam_pitch_deg 默认 1.3° (= gcs_aruco_det2 的 CAM_PITCH_DOWN_DEG, 同一颗相机;
  那是按右目标的, 左目差一个双目外参的小旋转, 用地面 --probe 复核)。
===============================================================================
"""

import os
import sys

# V4L2 select 超时 (秒): 默认 10s, 拔线后要等 10s 才知道。必须在 import cv2 之前设。
os.environ.setdefault("OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT", "2")

# ★ cv2 必须是**第一个**导入的第三方库 (2026-09-11 备用机实测):
#   rospy/zmq 先加载会吃掉 static TLS, 之后 cv2 的 libgomp 报
#   "cannot allocate memory in static TLS block"。单独 import cv2 永远正常, 别往下挪。
try:
    import cv2
except ImportError as _e:
    sys.exit("需要 opencv (cv2): %s" % _e)

import glob
import json
import subprocess
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from std_msgs.msg import String

try:
    import zmq
except ImportError:
    sys.exit("需要 pyzmq:  pip3 install pyzmq")
try:
    import msgpack

    def pack(o):
        return msgpack.packb(o, use_bin_type=True)

    def unpack(b):
        return msgpack.unpackb(b, raw=False)
except ImportError:
    sys.exit("需要 msgpack:  pip3 install msgpack")


# ============================== 常量 ==============================
UP_PORT, DOWN_PORT = 5555, 5556
POSE_TOPIC = "/mavros/local_position/pose"

# 新相机 1920×1080 档的标定内参 (同学 stereo_calib v2, 2026-10-08, rms_stereo 0.811px, 右目 K2)。
# 地面站 gcs_aruco_det2 自带同一组数并自己去畸变; 这里只是放进包头给别的消费者参考。
CAL_W, CAL_H = 1920, 1080
CAL_FX, CAL_FY, CAL_CX, CAL_CY = 1573.80, 1578.16, 966.15, 513.88

# ★ 上行两道门 (2026-09-16 实测, 原理不变):
#   ZMQ 的 SNDHWM 只管 ZMQ 自己的队列; 字节一旦进内核 SO_SNDBUF (会自增长到 ~500KB)
#   接收端 CONFLATE 也救不回来 → 延迟随时间单调增长。所以内核缓冲也钉死:
#   65536 → 内核翻倍 = 128KB。宁可丢帧, 不发旧帧。
SNDBUF_BYTES = 65536
SNDHWM_MSGS = 2

LEGACY_PARAMS = ("cam_topic", "caminfo_topic", "tx_width", "jpeg_quality")
OLD_CAM_MARK = "SPCA2100"           # 老相机 (VINS 用) 的 by-id 名字里有它, 绝不打开


# ============================== 小工具 ==============================
def find_device():
    """按 /dev/v4l/by-id 找新相机 (重新枚举后 videoN 会变, by-id 不变)。video-index0 才是图像节点。"""
    for pat in ("*STYT*video-index0", "*USB_Camera*video-index0", "*STYT*", "*USB_Camera*"):
        hits = [p for p in sorted(glob.glob("/dev/v4l/by-id/" + pat)) if OLD_CAM_MARK not in p]
        if hits:
            return hits[0]
    return ""


def legacy_procs():
    """还在跑的旧抓帧节点 / 看门狗 (它们会和本程序抢同一个相机)。"""
    out = []
    for name in ("det_cam_watchdog", "det_cam_node"):
        try:
            pids = subprocess.check_output(["pgrep", "-f", name + ".py"], text=True).split()
        except Exception:
            continue
        for p in pids:
            try:
                with open("/proc/%s/comm" % p) as f:
                    if f.read().strip().startswith("python"):
                        out.append("%s(pid %s)" % (name, p))
            except Exception:
                pass
    return out


def as_jpeg(buf):
    """
    retrieve() 的结果 → (jpeg_bytes | None, bgr_image | None)。
      CONVERT_RGB=0 生效: 一维 uint8 原始 MJPG 缓冲 (尾部可能有填充, 截到 FFD9) → (bytes, None)
      驱动不给原始数据 (返回已解码的 BGR): → (None, img), 调用方重编码
    """
    if buf is None:
        return None, None
    a = np.asarray(buf)
    if a.ndim == 3 or (a.ndim == 2 and a.shape[0] > 1 and a.shape[1] > 1):
        return None, a
    b = a.reshape(-1).tobytes()
    if len(b) < 4 or b[:2] != b"\xff\xd8":
        return None, None
    end = b.rfind(b"\xff\xd9")
    return (b[:end + 2] if end > 0 else b), None


# ============================== 相机 ==============================
class NewCam(object):
    """
    采集线程: 持续 grab (只出队不解码) 让驱动里永远只有最新帧;
    距上次交出 ≥ min_dt 才 retrieve 一帧, 存进 latest 并唤醒上行线程。
    掉线 (连续 grab 失败 / 太久没帧) → 自己 release + 重新找设备 + 重开。
    """

    FAIL_REOPEN = 3          # 连续 grab 失败次数
    STALL_REOPEN_S = 2.0     # 这么久没成功 grab 就重开

    def __init__(self, device, w, h, fps, out_hz):
        self.dev_param = device
        self.w, self.h, self.fps = int(w), int(h), float(fps)
        self.min_dt = 0.9 / float(out_hz)         # 放行正常频率, 挡住异常突发
        self.lock = threading.Lock()
        self.slots, self.events = {}, {}          # 每个消费者一个"最新帧"槽 (上行 / 检测)
        self.event = self.register("up")
        self.cap = None
        self.dev = ""
        self.ok = False
        self.err = "未打开"
        self.reopens = 0
        self.raw_supported = None                 # 驱动给不给原始 MJPG (第一帧后知道)
        self.actual_wh = None
        self.n_grab = 0
        self.t_last_grab = 0.0
        self._fails = 0

    def register(self, name):
        """新增一个消费者, 返回它的唤醒事件。每个消费者各自拿最新帧, 互不抢。"""
        with self.lock:
            self.slots[name] = None
            self.events[name] = threading.Event()
        return self.events[name]

    # ---- 打开 / 关闭 ----
    def _open(self):
        dev = self.dev_param or find_device()
        if not dev or not os.path.exists(dev):
            self.err = "找不到新相机 (/dev/v4l/by-id/*STYT*) —— USB 插好了吗? ls /dev/v4l/by-id/"
            return False
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            lp = legacy_procs()
            self.err = ("打不开 %s" % dev) + (
                " —— 被占用: %s 还在跑, 先停掉它 (run_vision.sh 会自动停)" % ", ".join(lp) if lp
                else " (被别的进程占用? fuser %s)" % dev)
            cap.release()
            return False
        # 顺序: 先 FOURCC 再尺寸 (部分 UVC 驱动反过来会退回 YUYV; USB2 下 YUYV 1080p 只有 1~3fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)      # ★ 拿原始 MJPG, 不解码
        self.cap, self.dev = cap, dev
        self._fails = 0
        self.t_last_grab = time.time()
        self.actual_wh = None
        return True

    def _close(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None

    def _reopen(self, why):
        self.ok = False
        self.err = why
        rospy.logwarn("[cam] ★ %s —— 重开相机" % why)
        self._close()
        self.reopens += 1

    # ---- 采集循环 ----
    def run(self):
        last_out = 0.0
        while not rospy.is_shutdown():
            if self.cap is None:
                if not self._open():
                    rospy.logwarn_throttle(5.0, "[cam] %s (每秒重试)" % self.err)
                    time.sleep(1.0)
                    continue
                rospy.loginfo("[cam] 已打开 %s  请求 %dx%d MJPG @%gfps" % (self.dev, self.w, self.h, self.fps))

            if not self.cap.grab():
                self._fails += 1
                if self._fails >= self.FAIL_REOPEN or time.time() - self.t_last_grab > self.STALL_REOPEN_S:
                    self._reopen("连续 %d 次取帧失败 / %.1fs 没有新帧 (USB 松了?)"
                                 % (self._fails, time.time() - self.t_last_grab))
                    time.sleep(0.5)
                continue
            stamp = rospy.Time.now()               # ★ 帧到手的时刻 = t_cap (飞机时钟)
            now = time.time()
            self._fails = 0
            self.t_last_grab = now
            self.n_grab += 1
            if now - last_out < self.min_dt:
                continue                           # 只出队, 不解码, 不交出

            ok, buf = self.cap.retrieve()
            raw, bgr = as_jpeg(buf if ok else None)
            if raw is None and bgr is None:
                continue
            if self.raw_supported is None:
                self.raw_supported = raw is not None
                if not self.raw_supported:
                    rospy.logwarn("[cam] 驱动没给原始 MJPG (CONVERT_RGB=0 不生效) —— 改为每帧重编码, "
                                  "多吃 CPU、多几十 ms")
            if self.actual_wh is None:            # 新开后第一帧: 核对分辨率 (只解一次, 缩小解码)
                if bgr is not None:
                    self.actual_wh = (bgr.shape[1], bgr.shape[0])
                else:
                    g = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_4)
                    self.actual_wh = (g.shape[1] * 4, g.shape[0] * 4) if g is not None else (0, 0)
                if self.actual_wh != (self.w, self.h):
                    rospy.logerr("[cam] ★ 实际分辨率 %dx%d ≠ 请求/标定的 %dx%d —— 内参会对不上, 测距错! "
                                 "v4l2-ctl -d %s --list-formats-ext"
                                 % (self.actual_wh + (self.w, self.h, self.dev)))
                else:
                    rospy.loginfo("[cam] 分辨率确认 %dx%d" % self.actual_wh)
            last_out = now
            self.ok, self.err = True, ""
            with self.lock:
                for k in self.slots:
                    self.slots[k] = (raw, bgr, stamp)
            for ev in list(self.events.values()):
                ev.set()
        self._close()

    def take(self, name="up"):
        with self.lock:
            item, self.slots[name] = self.slots.get(name), None  # 消费掉: 宁可少发也不发旧的
        return item


# ============================== gate 模式 ==============================
def quat_to_R(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def make_T_BC(pitch_down_deg, roll_deg, yaw_deg, lever):
    """
    原始左目相机(光学系: x右 y下 z前) → 机体(mavros base_link, FLU: x前 y左 z上)。
    全 0 = 相机朝正前方、水平安装。pitch_down 正 = 镜头往下低; roll 正 = 绕机体 x 右滚; yaw 正 = 往左偏。
    lever = 左目光心在机体系的位置 (m, FLU)。注意: 左目, 不是 1080p 档用的右目 (差一个基线 6cm)。
    """
    R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # 列 = 相机 x/y/z 轴在机体系
    p, r, yw = np.radians([pitch_down_deg, roll_deg, yaw_deg])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Rz = np.array([[np.cos(yw), -np.sin(yw), 0], [np.sin(yw), np.cos(yw), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Rx @ Ry @ R0
    T[:3, 3] = lever
    return T


class PoseBuffer(object):
    """/mavros/local_position/pose 的环形缓冲, 按 t_cap 插值 (位置线性 + 四元数 nlerp)。"""

    def __init__(self, n=300, max_gap=0.1, max_extrap=0.05):
        self.buf = []
        self.n, self.max_gap, self.max_extrap = n, max_gap, max_extrap
        self.lock = threading.Lock()

    def add(self, m):
        p, q = m.pose.position, m.pose.orientation
        item = (m.header.stamp.to_sec(), np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
        with self.lock:
            if self.buf and item[0] <= self.buf[-1][0]:
                return
            self.buf.append(item)
            if len(self.buf) > self.n:
                del self.buf[0]

    def at(self, t):
        """→ (4x4 T_odom←body, 说明)。拿不到可靠位姿返回 (None, 原因)。"""
        with self.lock:
            b = list(self.buf)
        if not b:
            return None, "no_pose"
        if t > b[-1][0]:
            if t - b[-1][0] > self.max_extrap:
                return None, "pose_late(%.0fms)" % ((t - b[-1][0]) * 1e3)
            t0, p, q = b[-1]
        elif t < b[0][0]:
            return None, "pose_too_old"
        else:
            i = int(np.searchsorted([x[0] for x in b], t))
            (ta, pa, qa), (tb, pb, qb) = b[max(i - 1, 0)], b[i]
            if tb - ta > self.max_gap:
                return None, "pose_gap(%.0fms)" % ((tb - ta) * 1e3)
            a = 0.0 if tb <= ta else (t - ta) / (tb - ta)
            p = pa + a * (pb - pa)
            if qa @ qb < 0:
                qb = -qb
            q = qa + a * (qb - qa)
        T = np.eye(4)
        T[:3, :3] = quat_to_R(*q)
        T[:3, 3] = p
        return T, "ok"


class GateOut(object):
    """
    gate / gate_gcs 共用的发布器: /track/target  /gate/target  /gate/status。
    frame_id: 飞机 _gate_sim 或 (地面检测时) 地面 sim 任一为真 → "odom_sim" (stage11 不响应)。
    """

    def __init__(self, sim, lost_timeout):
        self.sim, self.lost_timeout = bool(sim), float(lost_timeout)
        self.pub_t = rospy.Publisher("/track/target", PointStamped, queue_size=1)
        self.pub_c = rospy.Publisher("/gate/target", PoseWithCovarianceStamped, queue_size=1)
        self.pub_s = rospy.Publisher("/gate/status", String, queue_size=1)
        self.n, self.ms_sum, self.last = 0, 0.0, {}

    def emit(self, fr, t, pose_why, ms, sim_extra=False, extra=None):
        tg = fr.target
        sim = self.sim or bool(sim_extra)
        st = {"t": t, "stage": fr.stage, "pose": pose_why, "ms": round(float(ms), 1),
              "det_ms": {k: round(float(v), 1) for k, v in fr.t_ms.items()},
              "ring": {"ok": fr.ring.ok, "src": fr.ring.src, "partial": fr.ring.partial,
                       "corners": fr.ring.n_corners, "side": fr.ring.side},
              "dots": [d.src for d in fr.dots if d.ok], "used": fr.used, "rejected": fr.rejected,
              "skip_ring": fr.skipped_ring, "diag": fr.diag, "sim": sim}
        if extra:
            st.update(extra)
        publish = tg.valid and tg.age <= self.lost_timeout and fr.stage not in ("lost", "search")
        if tg.valid:
            sd = np.sqrt(np.diag(tg.cov_w))
            st.update(target=[round(float(v), 4) for v in tg.pos_w], dist=round(tg.dist, 3),
                      sd_cm=[round(float(v) * 100, 1) for v in sd], src=tg.src, age_ms=round(tg.age * 1000),
                      side=tg.side, published=publish,
                      # INTERFACE_gate §1: 世界系协方差 m² (行主序 3×3)。stage12 拿它把 σ 拆到靶面
                      cov=[float("%.4g" % v) for v in np.asarray(tg.cov_w, float).reshape(-1)[:9]])
        if publish:
            hdr_stamp = rospy.Time.from_sec(t)
            frame = "odom_sim" if sim else "odom"
            m = PointStamped()
            m.header.stamp, m.header.frame_id = hdr_stamp, frame
            m.point.x, m.point.y, m.point.z = (float(v) for v in tg.pos_w)
            self.pub_t.publish(m)
            c = PoseWithCovarianceStamped()
            c.header.stamp, c.header.frame_id = hdr_stamp, frame
            c.pose.pose.position.x, c.pose.pose.position.y, c.pose.pose.position.z = (float(v) for v in tg.pos_w)
            c.pose.pose.orientation.w = 1.0
            cov = [0.0] * 36
            for i in range(3):
                for j in range(3):
                    cov[i * 6 + j] = float(tg.cov_w[i, j])
            for i in range(3, 6):
                cov[i * 6 + i] = 1e6               # 姿态无意义
            c.pose.covariance = cov
            self.pub_c.publish(c)
        self.pub_s.publish(String(data=json.dumps(st, default=str)))
        self.n += 1
        self.ms_sum += st["ms"]
        self.last = st
        return st

    def pop_stats(self):
        n, ms = self.n, self.ms_sum
        self.n, self.ms_sum = 0, 0.0
        return n, (ms / n if n else None), self.last


def _load_gate_cfg(gd, cfg, here):
    params = cfg["gate_params"] if os.path.isabs(cfg["gate_params"]) else os.path.join(here, cfg["gate_params"])
    if os.path.exists(params):
        return gd.GateConfig.load(params), params
    return gd.GateConfig(), "(默认参数, 没找到 %s)" % params


def _reset_age(cfg, gcfg):
    a = float(cfg.get("gate_reset_age", 0.0))
    return a if a > 0 else max(1.0, float(getattr(gcfg, "lost_timeout", 1.0)))


def _ring_extra(gd, fuser, fr, T_WC):
    """
    ★2026-10-19 按环心穿 (stage12): 估计器的 target 是**红点**; 红点没定位上时它是"环心 + 0.3m 先验"
    (ring_prior), σ 停在 6~7cm 压不下去。所以另发这一帧的环心观测, 让 stage12 自己多帧取中值:
      ring_w    这一帧的环心 (world)             ring_cov  它的协方差 (m², 行主序 3×3)
      ring_src  stereo (角点平面) / mono (椭圆)   ring_c    估计器累积的环心 (world, 显示用)
      n         估计器的环法向 (world, 已翻成指向相机一侧; 至少 3 次可信法向才有)
      aim       红点 (world) —— 只有估计器的来源是 dot_* 时才发
    """
    out = {}
    try:
        R_WR, t_WR = fuser.T_WR(T_WC)
        if R_WR is None:
            return out
        est, ring = fuser.est, fr.ring
        if ring.ok and ring.c is not None:
            cw = R_WR @ ring.c + t_WR
            C = R_WR @ gd._cov_along_ray(ring.c, max(ring.sigma_lat, 1e-3), max(ring.sigma_z, 1e-3)) @ R_WR.T
            out.update(ring_w=[round(float(v), 4) for v in cw], ring_src=ring.src,
                       ring_cov=[float("%.4g" % v) for v in C.reshape(-1)],
                       ring_partial=bool(ring.partial))
        if est.ring_c is not None:
            out["ring_c"] = [round(float(v), 4) for v in est.ring_c]
        n = est.ring_n
        if n is not None and est.ring_c is not None:
            n = np.asarray(n, float)
            if (t_WR - est.ring_c) @ n < 0:
                n = -n
            out["n"] = [round(float(v), 5) for v in n]
        tg = fr.target
        if tg is not None and tg.valid and str(tg.src).startswith("dot"):
            out["aim"] = [round(float(v), 4) for v in tg.pos_w]
    except Exception as e:                       # 附加字段出错不能影响主链路
        rospy.logwarn_throttle(10.0, "[gate] ring 附加字段出错: %r" % (e,))
    return out


def _stale(fr, reset_age):
    """这一帧之后估计器里还有一个**旧**的环 (valid 但 age > reset_age) → True。"""
    tg = getattr(fr, "target", None)
    return tg is not None and bool(getattr(tg, "valid", False)) and float(getattr(tg, "age", 0.0)) > reset_age


class GateWorker(object):
    """检测线程: 拿相机最新帧 → 降采样解码 → 校正 → gate_detector → 发布。只处理最新帧, 不排队。"""

    def __init__(self, cfg, cam):
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)
        import ring_detector as rd          # 只依赖 cv2/numpy, 不影响上面 "cv2 先导入" 的约束
        import gate_detector as gd
        self.gd, self.cfg, self.cam = gd, cfg, cam
        calib = cfg["calib"] if os.path.isabs(cfg["calib"]) else os.path.join(here, cfg["calib"])
        self.rig = rd.StereoRig.from_file(calib, (cfg["cam_w"] // 2, cfg["cam_h"]), cfg["det_scale"])
        gcfg, src = _load_gate_cfg(gd, cfg, here)
        self.gcfg = gcfg
        self.det = gd.GateDetector(self.rig, gcfg)
        self.reset_age = _reset_age(cfg, gcfg)
        self.n_reset = 0
        self.T_BC = make_T_BC(cfg["cam_pitch_deg"], cfg["cam_roll_deg"], cfg["cam_yaw_deg"],
                              [cfg["cam_x"], cfg["cam_y"], cfg["cam_z"]])
        self.poses = PoseBuffer(max_gap=cfg["max_pose_gap"])
        self.ev = cam.register("det")
        self.min_dt = 1.0 / max(cfg["det_hz"], 1e-3)
        self.prev_lock = threading.Lock()
        self.preview = None                 # (jpeg, w, h, stamp, status) 给上行线程
        self.prev_ev = threading.Event()
        self.last_prev = 0.0
        self.out = GateOut(cfg["gate_sim"], gcfg.lost_timeout)
        flag = cv2.IMREAD_REDUCED_COLOR_2 if abs(cfg["det_scale"] - 0.5) < 1e-6 else \
            cv2.IMREAD_REDUCED_COLOR_4 if abs(cfg["det_scale"] - 0.25) < 1e-6 else cv2.IMREAD_COLOR
        self.dec_flag = flag
        rospy.loginfo("[gate] 标定 %s → 工作分辨率 %dx%d/目 f=%.1f B=%.1fmm (T 单位 %s)" %
                      (calib, self.rig.size[0], self.rig.size[1], self.rig.f, self.rig.B * 1000,
                       getattr(self.rig, "T_unit", "?")))
        rospy.loginfo("[gate] 参数 %s" % src)
        rospy.loginfo("[gate] T_BC: pitch_down=%.1f° roll=%.1f° yaw=%.1f° lever=(%.3f,%.3f,%.3f)m  "
                      "frame_id=%s" % (cfg["cam_pitch_deg"], cfg["cam_roll_deg"], cfg["cam_yaw_deg"],
                                       cfg["cam_x"], cfg["cam_y"], cfg["cam_z"],
                                       "odom_sim (★不会驱动飞机)" if cfg["gate_sim"] else "odom (★会驱动飞机)"))

    def on_pose(self, m):
        self.poses.add(m)

    def take_preview(self):
        with self.prev_lock:
            item, self.preview = self.preview, None
        return item

    def run(self):
        last_t = 0.0
        while not rospy.is_shutdown():
            if not self.ev.wait(timeout=1.0):
                continue
            self.ev.clear()
            item = self.cam.take("det")
            if item is None:
                continue
            now = time.time()
            if now - last_t < self.min_dt * 0.9:
                continue
            last_t = now
            try:
                self.step(*item)
            except Exception as e:
                rospy.logwarn_throttle(2.0, "[gate] 处理出错: %r" % (e,))

    def step(self, raw, bgr, stamp):
        t0 = time.time()
        img = cv2.imdecode(np.frombuffer(raw, np.uint8), self.dec_flag) if raw is not None else bgr
        if img is None:
            return
        L, R = self.rig.rectify(*self.rig.split(img))
        t = stamp.to_sec()
        T_WB, why = self.poses.at(t)
        T_WC = T_WB @ self.T_BC if T_WB is not None else None
        fr = self.det.process(L, R, t, T_WC)
        if _stale(fr, self.reset_age):
            # 估计器拿着一个 reset_age 没更新过的环 (落地后 / VINS 跳过 / 新测量一直被当野值拒绝)
            # → 整个检测器重建, 用这一帧重新初始化 (没看到环就是 "没有环", 不再发旧值)
            self.n_reset += 1
            rospy.logwarn("[gate] 环估计 %.1fs 没更新 → 重置 (第 %d 次)" % (fr.target.age, self.n_reset))
            self.det = self.gd.GateDetector(self.rig, self.gcfg)
            fr = self.det.process(L, R, t, T_WC)
        st = self.out.emit(fr, t, why, (time.time() - t0) * 1000,
                           extra=_ring_extra(self.gd, self.det.fuser, fr, T_WC))
        # 给上行线程的预览 (按 tx_hz 节流, 只在需要时画)
        if self.cfg["tx_hz"] > 0 and time.time() - self.last_prev >= 0.9 / self.cfg["tx_hz"]:
            self.last_prev = time.time()
            vis = self.gd.draw_frame(L, fr, self.det, T_WC)
            ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), self.cfg["jpeg_q"]])
            if ok:
                with self.prev_lock:
                    self.preview = (buf.tobytes(), vis.shape[1], vis.shape[0], stamp, st)
                self.prev_ev.set()

    def pop_stats(self):
        return self.out.pop_stats()


class GateFuseWorker(object):
    """
    gate_gcs 模式的飞机侧: 地面 gcs_gate.py 回的是相机(校正左目)系观测,
    这里按 t_cap 插值位姿 → GateFuser (估计器) → 发布。在下行线程里调用, 很轻 (几个 3x3 矩阵)。
    """

    def __init__(self, cfg):
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)
        import gate_detector as gd          # 只依赖 cv2/numpy
        self.gd, self.cfg = gd, cfg
        self.gcfg, src = _load_gate_cfg(gd, cfg, here)
        self.T_BC = make_T_BC(cfg["cam_pitch_deg"], cfg["cam_roll_deg"], cfg["cam_yaw_deg"],
                              [cfg["cam_x"], cfg["cam_y"], cfg["cam_z"]])
        self.poses = PoseBuffer(max_gap=cfg["max_pose_gap"])
        self.out = GateOut(cfg["gate_sim"], self.gcfg.lost_timeout)
        self.fuser = None
        self.rig = None
        self.reset_age = _reset_age(cfg, self.gcfg)
        self.n_reset = 0
        self.lock = threading.Lock()
        self.status = None                  # 最新融合结果, 放进上行包头给地面显示
        self.n_bad = 0
        rospy.loginfo("[gate_gcs] 检测在地面 (gcs_gate.py); 飞机只做位姿插值 + 融合。估计器参数 %s" % src)
        rospy.loginfo("[gate_gcs] T_BC: pitch_down=%.1f° roll=%.1f° yaw=%.1f° lever=(%.3f,%.3f,%.3f)m  "
                      "飞机侧保险 %s" % (cfg["cam_pitch_deg"], cfg["cam_roll_deg"], cfg["cam_yaw_deg"],
                                        cfg["cam_x"], cfg["cam_y"], cfg["cam_z"],
                                        "odom_sim (★强制锁住, 地面 --mode 2 也不会驱动飞机)" if cfg["gate_sim"]
                                        else "跟随地面 --mode (2 = ★驱动飞机)"))

    def on_pose(self, m):
        self.poses.add(m)

    def _rig_ok(self, rig):
        try:
            f = float(rig["f"])
            R1 = np.asarray(rig["R1"], float).reshape(3, 3)
        except Exception:
            return None
        if not (np.isfinite(f) and f > 0 and np.all(np.isfinite(R1))):
            return None
        return f, R1

    def on_obs(self, det):
        """处理一个下行包。返回 True = 用上了。"""
        g, rig, tc = det.get("gate"), det.get("rig"), det.get("t_cap")
        fr_rig = self._rig_ok(rig) if isinstance(rig, dict) else None
        if not isinstance(g, dict) or fr_rig is None or tc is None:
            self.n_bad += 1
            rospy.logwarn_throttle(5.0, "[gate_gcs] 回包缺 gate/rig/t_cap 字段 (地面程序版本不对?) —— 丢弃")
            return False
        f, R1 = fr_rig
        if self.fuser is None or not self.fuser.same_rig(f, R1):
            if self.fuser is not None:
                rospy.logwarn("[gate_gcs] ★ 地面的标定变了 (f %.1f → %.1f) —— 估计器重置" % (self.fuser.f, f))
            self.fuser = self.gd.GateFuser(self.gcfg, f, R1)
            self.rig = dict(rig)
            rospy.loginfo("[gate_gcs] 地面标定: f=%.1f B=%.1fmm 工作分辨率 %s/目"
                          % (f, float(rig.get("B") or 0) * 1000, rig.get("size")))
        t = float(tc)
        fr = self.gd.frame_from_msg(g)
        T_WB, why = self.poses.at(t)
        T_WC = T_WB @ self.T_BC if T_WB is not None else None
        self.fuser.update(fr, T_WC, t)
        if _stale(fr, self.reset_age):
            # 同 GateWorker: 旧状态 → 重建估计器, 用这一帧的观测重新初始化
            self.n_reset += 1
            rospy.logwarn("[gate_gcs] 环估计 %.1fs 没更新 → 重置 (第 %d 次)" % (fr.target.age, self.n_reset))
            self.fuser = self.gd.GateFuser(self.gcfg, f, R1)
            fr = self.gd.frame_from_msg(g)
            self.fuser.update(fr, T_WC, t)
        ground_sim = bool(det.get("sim", True))
        ex = {"ground_sim": ground_sim, "ground_stage": g.get("stage")}
        ex.update(_ring_extra(self.gd, self.fuser, fr, T_WC))
        st = self.out.emit(fr, t, why, fr.t_ms.get("total", 0.0), sim_extra=ground_sim, extra=ex)
        with self.lock:
            self.status = st
        return True

    def head_status(self):
        with self.lock:
            return self.status

    def pop_stats(self):
        return self.out.pop_stats()


# ============================== 往返时间 ==============================
class RttStat(object):
    """收到回包时 now - t_cap (都是飞机时钟, 不需要对时)。对**每个**回包都测, 和 found 无关。"""
    WIN, STALL_MS = 200, 800.0

    def __init__(self):
        self.v, self.n, self.stall, self._in = [], 0, 0, False
        self.last = self.p50 = self.p95 = float("nan")
        self.max = 0.0

    def add(self, ms):
        self.last, self.n = ms, self.n + 1
        self.max = max(self.max, ms)
        self.v.append(ms)
        if len(self.v) > self.WIN:
            del self.v[0]
        if ms > self.STALL_MS:
            if not self._in:
                self.stall += 1
            self._in = True
        else:
            self._in = False
        if len(self.v) >= 5:
            sv = sorted(self.v)
            self.p50, self.p95 = sv[len(sv) // 2], sv[min(len(sv) - 1, int(0.95 * len(sv)))]

    def as_head(self):
        return {"rtt_ms": self.last, "rtt_p50": self.p50, "rtt_p95": self.p95,
                "rtt_max": self.max, "rtt_n": self.n, "rtt_stall": self.stall}


# ============================== 桥 ==============================
class Bridge(object):
    def __init__(self, cfg, cam, gate=None, fuse=None):
        self.cfg, self.cam, self.gate, self.fuse = cfg, cam, gate, fuse
        self.lock = threading.Lock()
        self.pose = self.state = self.phase = None
        self.seq = 0
        self.rtt = RttStat()
        self.stats = {"tx": 0, "rx": 0, "found": 0, "drop": 0, "raw": 0, "reenc": 0, "gate": 0, "kb": 0.0}

    # ---- ROS 回调 ----
    def on_pose(self, m):
        with self.lock:
            self.pose = m
        if self.gate is not None:
            self.gate.on_pose(m)
        if self.fuse is not None:
            self.fuse.on_pose(m)

    def on_state(self, m):
        with self.lock:
            self.state = m

    def on_phase(self, m):
        with self.lock:
            self.phase = m.data

    # ---- 编码 ----
    def encode(self, raw, bgr):
        """→ (jpeg, w, h, mode)。能原样转发就原样转发。"""
        c = self.cfg
        native_w = self.cam.actual_wh[0] if self.cam.actual_wh else CAL_W
        need_shrink = c["tx_max_w"] and native_w > c["tx_max_w"]
        if raw is not None and not need_shrink and c["raw_max_kb"] > 0 \
                and len(raw) <= c["raw_max_kb"] * 1024:
            w, h = self.cam.actual_wh or (CAL_W, CAL_H)
            return raw, w, h, "raw"
        img = bgr
        if img is None:
            flag = cv2.IMREAD_COLOR
            if need_shrink and c["tx_max_w"] <= native_w // 2:
                flag = cv2.IMREAD_REDUCED_COLOR_2          # 解码时直接缩一半, 快得多
            img = cv2.imdecode(np.frombuffer(raw, np.uint8), flag)
            if img is None:
                return None, 0, 0, ""
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        h0, w0 = img.shape[:2]
        if c["tx_max_w"] and w0 > c["tx_max_w"]:
            img = cv2.resize(img, (c["tx_max_w"], int(round(h0 * c["tx_max_w"] / float(w0)))),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), c["jpeg_q"]])
        if not ok:
            return None, 0, 0, ""
        return buf.tobytes(), img.shape[1], img.shape[0], "reenc"

    # ---- 上行 ----
    def uplink_loop(self, sock):
        gate = self.gate
        ev = gate.prev_ev if gate is not None else self.cam.event
        while not rospy.is_shutdown():
            if not ev.wait(timeout=1.0):
                continue
            ev.clear()
            gst = None
            if gate is not None:                      # gate 模式: 发校正后左目 + 叠加 (只给人看)
                item = gate.take_preview()
                if item is None:
                    continue
                jpg, w, h, stamp, gst = item
                mode = "gate"
            else:
                item = self.cam.take()
                if item is None:
                    continue
                raw, bgr, stamp = item
            with self.lock:
                pose, state, ph = self.pose, self.state, self.phase
            try:
                if gate is None:
                    jpg, w, h, mode = self.encode(raw, bgr)
                    if jpg is None:
                        continue
                if gate is not None:
                    K = [gate.rig.f, gate.rig.f, gate.rig.cx, gate.rig.cy]   # 校正后左目
                elif self.fuse is not None:
                    K = None                       # 并排帧: 地面用自己的双目标定
                    gst = self.fuse.head_status()  # 飞机最新融合结果, 地面只显示
                else:
                    s = w / float(CAL_W)
                    K = [CAL_FX * s, CAL_FY * s, CAL_CX * s, CAL_CY * s]
                self.seq += 1
                head = {
                    "seq": self.seq,
                    "t_cap": stamp.to_sec(),                  # 飞机时钟, 地面站原样回填
                    "w": w, "h": h, "fmt": "jpeg",
                    "src": "gate_left_rect" if gate else ("newcam_sbs" if self.fuse else "newcam"),
                    "sbs": self.fuse is not None,          # 并排双目 (gate_gcs)
                    "tx_mode": mode, "K": K,
                    "ground_z": self.cfg["ground_z"],
                    "pose": None, "fc": None, "ph": ph, "gate": gst,
                    "jpg": jpg,
                }
                head.update(self.rtt.as_head())
                if state is not None:
                    head["fc"] = {"mode": state.mode, "armed": bool(state.armed),
                                  "connected": bool(state.connected)}
                if pose is not None:
                    p, q = pose.pose.position, pose.pose.orientation
                    head["pose"] = {"p": [p.x, p.y, p.z], "q": [q.x, q.y, q.z, q.w]}
                sock.send(pack(head), flags=zmq.NOBLOCK)     # 发不出去就丢, 绝不排队
                self.stats["tx"] += 1
                self.stats[mode] = self.stats.get(mode, 0) + 1
                self.stats["kb"] += len(jpg) / 1024.0
            except zmq.Again:
                self.stats["drop"] += 1
            except Exception as e:
                rospy.logwarn_throttle(5.0, "[bridge] uplink: %s" % e)

    # ---- 下行 ----
    def downlink_loop(self, sock, pub, rtt_pub):
        while not rospy.is_shutdown():
            try:
                if not sock.poll(timeout=200):
                    continue
                det = unpack(sock.recv(flags=zmq.NOBLOCK))
            except zmq.Again:
                continue
            except Exception as e:
                rospy.logwarn_throttle(5.0, "[bridge] downlink: %s" % e)
                continue
            self.stats["rx"] += 1
            tc = det.get("t_cap")
            if tc is not None:
                try:
                    ms = (rospy.Time.now().to_sec() - float(tc)) * 1000.0
                    if ms >= 0.0:
                        self.rtt.add(ms)
                        r = self.rtt
                        rtt_pub.publish(String(data="rtt=%.0f p50=%.0f p95=%.0f max=%.0f n=%d stall=%d"
                                               % (r.last, r.p50, r.p95, r.max, r.n, r.stall)))
                except Exception:
                    pass
            if self.fuse is not None:              # gate_gcs: 只认地面的相机系观测
                if det.get("mode") == 0:           # 地面 --mode 0: 不检测, 只为往返统计回包
                    continue
                if "gate" in det:
                    self.stats["found"] += int(bool(det.get("found")))
                    try:
                        self.fuse.on_obs(det)
                    except Exception as e:
                        rospy.logwarn_throttle(2.0, "[gate_gcs] 融合出错: %r" % (e,))
                else:
                    rospy.logwarn_throttle(10.0, "[bridge] gate_gcs 模式: 回包里没有 gate 观测 —— 地面跑的是 "
                                                 "gcs_aruco_det2? 换成 gcs_gate.py (这种回包一律不发布)")
                continue
            if not det.get("found"):
                continue
            if self.gate is not None:
                rospy.logwarn_throttle(10.0, "[bridge] gate 模式: 忽略地面站回包里的 target "
                                             "(/track/target 由机上 gate 检测发布)")
                continue
            tgt = det.get("target")
            if tgt is None or tc is None:
                continue
            self.stats["found"] += 1
            m = PointStamped()
            m.header.stamp = rospy.Time(float(tc))           # 关键: 拍摄时刻
            m.header.frame_id = "odom_sim" if det.get("sim") else "odom"   # _sim → 飞机侧强制增益 0
            m.point.x, m.point.y = float(tgt[0]), float(tgt[1])
            m.point.z = float(tgt[2]) if len(tgt) > 2 else self.cfg["ground_z"]
            pub.publish(m)

    # ---- 统计 + 状态话题 ----
    def stats_loop(self, cam_pub):
        last = dict(self.stats)
        last_grab, t_last = self.cam.n_grab, time.time()
        n = 0
        while not rospy.is_shutdown():
            time.sleep(1.0)
            now = time.time()
            dt = max(now - t_last, 1e-3)
            d = {k: self.stats[k] - last[k] for k in self.stats}
            grabs = self.cam.n_grab - last_grab
            last, last_grab, t_last = dict(self.stats), self.cam.n_grab, now
            age = now - self.cam.t_last_grab if self.cam.t_last_grab else None
            ok, err = self.cam.ok, self.cam.err
            if ok and (age is None or age > 1.0):          # 还没到重开门限, 但已经断流了
                ok, err = False, "%.1fs 没有新帧" % (age or 0.0)
            st = {
                "t": now, "ok": ok, "err": err, "dev": self.cam.dev,
                "w": (self.cam.actual_wh or (0, 0))[0], "h": (self.cam.actual_wh or (0, 0))[1],
                "cam_fps": round(grabs / dt, 1), "tx_hz": round(d["tx"] / dt, 1),
                "raw_pct": round(100.0 * d["raw"] / d["tx"], 0) if d["tx"] else None,
                "kb": round(d["kb"] / d["tx"], 1) if d["tx"] else None,
                "mbps": round(d["kb"] * 8 / 1024.0 / dt, 2),
                "drop": d["drop"], "rx_hz": round(d["rx"] / dt, 1),
                "reopens": self.cam.reopens, "raw_supported": self.cam.raw_supported,
                "frame_age": round(age, 2) if age is not None else None,
                "rtt_p50": self.rtt.p50 if self.rtt.n >= 5 else None, "rtt_stall": self.rtt.stall,
                "cfg": {k: self.cfg[k] for k in ("mode", "cam_fps", "tx_hz", "tx_max_w", "raw_max_kb", "jpeg_q")},
            }
            gw = self.gate or self.fuse
            if gw is not None:
                gn, gms, glast = gw.pop_stats()
                st["gate"] = {"hz": round(gn / dt, 1), "ms": round(gms, 1) if gms else None,
                              "stage": glast.get("stage"), "dist": glast.get("dist"),
                              "pose": glast.get("pose"), "src": glast.get("src")}
            try:
                cam_pub.publish(String(data=json.dumps(st, allow_nan=False, default=str)))
            except ValueError:
                cam_pub.publish(String(data=json.dumps(
                    {k: (None if isinstance(v, float) and v != v else v) for k, v in st.items()},
                    default=str)))
            n += 1
            if n % 2 == 0:
                rospy.loginfo("[bridge] 相机 %s %.1ffps | 上行 %.1fHz %s %.2fMbit/s 丢%d | 回包 %.1fHz | 重开 %d%s"
                              % ("✓" if st["ok"] else "✗", st["cam_fps"], st["tx_hz"],
                                 ("原样%d%% %.0fKB/张" % (st["raw_pct"], st["kb"])) if st["kb"] else "",
                                 st["mbps"], st["drop"], st["rx_hz"], st["reopens"],
                                 ("" if st["ok"] else "  ★" + st["err"])))
                if self.rtt.n >= 5:
                    rospy.loginfo("[bridge] 往返 p50=%.0f p95=%.0f max=%.0f ms 卡=%d (n=%d)"
                                  % (self.rtt.p50, self.rtt.p95, self.rtt.max, self.rtt.stall, self.rtt.n))
                if "gate" in st:
                    g_ = st["gate"]
                    rospy.loginfo("[gate%s] %.1fHz %sms | 阶段 %s | 距离 %s | 来源 %s | 位姿 %s"
                                  % ("_gcs 地面检测" if self.fuse else "", g_["hz"], g_["ms"], g_["stage"],
                                     g_["dist"], g_["src"], g_["pose"]))


# ============================== main ==============================
CFG_KEYS = ("mode", "gcs_host", "device", "cam_w", "cam_h", "cam_fps", "tx_hz", "tx_max_w",
            "raw_max_kb", "jpeg_q", "status_topic", "ground_z")
GATE_KEYS = ("det_scale", "det_hz", "calib", "gate_params", "gate_sim", "cam_pitch_deg", "cam_roll_deg",
             "cam_yaw_deg", "cam_x", "cam_y", "cam_z", "max_pose_gap", "gate_reset_age")


def drop_stale_params():
    """
    ★只认**这次命令行**上给的 _xxx:=值。
    rospy 会把命令行的 _xxx:=值 写进 ROS master 的 /vision_bridge/xxx, 进程退出后**还在**;
    下次不带这个参数启动, get_param 会读到上次的值 —— 就是旧版"参数残留"那个坑。
    所以: master 上有、但这次命令行没给的, 一律删掉, 用代码默认值。
    """
    given = set()
    for a in sys.argv[1:]:
        if a.startswith("_") and ":=" in a:
            given.add(a[1:a.index(":=")])
    me = rospy.get_name()                         # "/vision_bridge"
    try:
        ns = rospy.get_param(me, {})              # 取命名空间 → dict
    except Exception:
        ns = {}
    stale = sorted(k for k in (ns if isinstance(ns, dict) else {}) if k not in given)
    for k in stale:
        try:
            rospy.delete_param(me + "/" + k)
        except Exception:
            pass
    return stale


def read_cfg():
    """全部参数在 init_node **之后**读 (之前读, ~私有名解析不到 → 永远是默认值)。"""
    stale = drop_stale_params()
    if stale:
        rospy.logwarn("[bridge] 上次残留在 ROS master 的参数已删除 (这次命令行没给): %s"
                      % ", ".join(stale))
    g = rospy.get_param
    mode = str(g("~mode", "aruco")).strip().lower()
    if mode not in ("aruco", "gate", "gate_gcs"):
        rospy.logwarn("[bridge] 未知 mode=%s, 用 aruco" % mode)
        mode = "aruco"
    gate = mode in ("gate", "gate_gcs")
    cfg = {
        "mode": mode,
        "gcs_host": str(g("~gcs_host", "192.168.2.179")),
        "device": str(g("~device", "")),
        "cam_w": int(g("~cam_w", 2 * CAL_W if gate else CAL_W)), "cam_h": int(g("~cam_h", CAL_H)),
        "cam_fps": float(g("~cam_fps", 15.0 if gate else 10.0)),
        "tx_hz": float(g("~tx_hz", 15.0 if mode == "gate_gcs" else 10.0)),   # gate_gcs: 上行帧率 = 地面检测帧率
        "tx_max_w": int(g("~tx_max_w", CAL_W)),
        "raw_max_kb": float(g("~raw_max_kb", 260)),
        "jpeg_q": int(g("~jpeg_q", 85)),
        "status_topic": str(g("~status_topic", "/flight_ctrl/status")),
        "ground_z": float(g("~ground_z", 0.0)),
        # ---- gate 模式 ----
        "det_scale": float(g("~det_scale", 0.5)),        # 检测工作分辨率 (0.5 → 每目 960×540)
        "det_hz": float(g("~det_hz", 30.0)),             # 检测频率上限 (实际受 CPU 限制)
        "calib": str(g("~calib", "stereo_calib.yaml")),
        "gate_params": str(g("~gate_params", "gate_params.json")),
        # ★ gate (机上检测): 默认 True = frame_id odom_sim, 不驱动飞机 (没有地面开关, 只有这一道)
        #   gate_gcs: 默认 False = 跟随地面 --mode (单保险, 同 aruco 链路); 给 true 强制锁住
        "gate_sim": bool(g("~gate_sim", mode == "gate")),
        # 左目外参 ★2026-10-20 stage13_cam_calib 手持标定 (33 工位, 2.9~5.3m, 留一验证 竖直 5.1 / 横向 2.5cm):
        #   pitch_down 5.23±0.25° (原 1.3° 是右目 ArUco 标的, 少了 ~4° → 环越远估得越高)
        #   roll 3.40±0.31° (环都在画面中间, 不太可信, 影响 ≤2cm)  yaw -0.73±0.36°
        #   杠杆臂卷尺: 左目在飞控前 8cm / 左 3cm / 低 11.5cm (飞控离地 16cm, 识别相机离地 4.5cm)
        "cam_pitch_deg": float(g("~cam_pitch_deg", 5.23)),  # 正 = 镜头往下低
        "cam_roll_deg": float(g("~cam_roll_deg", 3.40)),
        "cam_yaw_deg": float(g("~cam_yaw_deg", -0.73)),
        "cam_x": float(g("~cam_x", 0.080)), "cam_y": float(g("~cam_y", 0.030)), "cam_z": float(g("~cam_z", -0.115)),
        "max_pose_gap": float(g("~max_pose_gap", 0.1)),
        # ★2026-10-19 (穿环首飞问题 §2): 环估计这么久没被新检测更新 → 丢弃, 从下一帧重新初始化。
        #   落地后 / VINS 跳过之后, 估计器会只 predict 不 update (年龄几百秒、σ 上米还在发)。
        #   ≤ 0 = 自动: max(1.0, gate_params 的 lost_timeout) —— 还在发布期内的估计不会被清
        "gate_reset_age": float(g("~gate_reset_age", 0.0)),
    }
    legacy = [k for k in LEGACY_PARAMS if ("_%s:=" % k) in " ".join(sys.argv)]
    if legacy:
        rospy.logwarn("[bridge] 命令行给了旧参数 %s —— 本版不用 (桥自己开新相机, 不订图像话题)"
                      % ", ".join(legacy))
    rospy.loginfo("[bridge] 生效参数: " + "  ".join("%s=%s" % (k, cfg[k]) for k in CFG_KEYS))
    if gate:
        rospy.loginfo("[bridge] gate 参数: " + "  ".join("%s=%s" % (k, cfg[k]) for k in GATE_KEYS))
    return cfg


def main():
    rospy.init_node("vision_bridge")
    cfg = read_cfg()

    lp = legacy_procs()
    if lp:
        rospy.logwarn("[bridge] ★ 旧的 %s 还在跑 —— 会和本程序抢相机。先停掉它 (run_vision.sh 会自动停)"
                      % ", ".join(lp))

    ctx = zmq.Context()
    up = ctx.socket(zmq.PUB)
    up.setsockopt(zmq.SNDHWM, SNDHWM_MSGS)
    up.setsockopt(zmq.SNDBUF, SNDBUF_BYTES)      # 必须在 bind 之前; 每条连接继承
    up.setsockopt(zmq.LINGER, 0)
    up.bind("tcp://*:%d" % UP_PORT)

    down = ctx.socket(zmq.SUB)
    down.setsockopt(zmq.RCVHWM, 1)
    down.setsockopt(zmq.CONFLATE, 1)             # 单帧消息才能用 CONFLATE
    down.setsockopt(zmq.LINGER, 0)
    down.setsockopt_string(zmq.SUBSCRIBE, "")
    down.connect("tcp://%s:%d" % (cfg["gcs_host"], DOWN_PORT))

    is_gate = cfg["mode"] == "gate"
    cam = NewCam(cfg["device"], cfg["cam_w"], cfg["cam_h"], cfg["cam_fps"],
                 max(cfg["tx_hz"], cfg["det_hz"]) if is_gate else cfg["tx_hz"])
    gate = fuse = None
    if is_gate:
        try:
            gate = GateWorker(cfg, cam)
        except Exception as e:
            rospy.logfatal("[gate] 初始化失败: %r —— 检查同目录有 gate_detector.py ring_detector.py "
                           "stereo_calib.yaml (gate_params.json 可选)" % (e,))
            sys.exit(1)
    elif cfg["mode"] == "gate_gcs":
        try:
            fuse = GateFuseWorker(cfg)
        except Exception as e:
            rospy.logfatal("[gate_gcs] 初始化失败: %r —— 检查同目录有 gate_detector.py ring_detector.py" % (e,))
            sys.exit(1)
        if cfg["tx_max_w"] < cfg["cam_w"] // 2:
            rospy.logwarn("[gate_gcs] tx_max_w=%d < %d: 上行并排帧每目窄于 960, 地面检测分辨率会低于机上一体版"
                          % (cfg["tx_max_w"], cfg["cam_w"] // 2))
    br = Bridge(cfg, cam, gate, fuse)

    rospy.loginfo("[bridge] 上行 bind *:%d  下行 connect %s:%d" % (UP_PORT, cfg["gcs_host"], DOWN_PORT))
    rospy.loginfo("[bridge] 新相机 %s  %dx%d @%gfps → 上行 ≤%gHz, 宽 ≤%d, 原始帧 ≤%gKB 原样转发, 否则 JPEG q%d"
                  % (cfg["device"] or "(自动 by-id)", cfg["cam_w"], cfg["cam_h"], cfg["cam_fps"],
                     cfg["tx_hz"], cfg["tx_max_w"], cfg["raw_max_kb"], cfg["jpeg_q"]))
    rospy.loginfo("[bridge] 阶段状态 ← %s  ·  相机状态 → /vision_bridge/cam (1Hz JSON)"
                  % cfg["status_topic"])
    rospy.loginfo("[bridge] 上行队列: SNDHWM=%d 内核 SNDBUF=%dB (实际 128KB)" % (SNDHWM_MSGS, SNDBUF_BYTES))

    if gate is not None:
        pub = gate.out.pub_t
    elif fuse is not None:
        pub = fuse.out.pub_t
    else:
        pub = rospy.Publisher("/track/target", PointStamped, queue_size=1)
    rtt_pub = rospy.Publisher("/bridge/rtt", String, queue_size=1)
    cam_pub = rospy.Publisher("/vision_bridge/cam", String, queue_size=1)
    rospy.Subscriber(POSE_TOPIC, PoseStamped, br.on_pose, queue_size=1)
    rospy.Subscriber("/mavros/state", State, br.on_state, queue_size=1)
    rospy.Subscriber(cfg["status_topic"], String, br.on_phase, queue_size=1)

    threads = [(cam.run, ()), (br.uplink_loop, (up,)),
               (br.downlink_loop, (down, pub, rtt_pub)), (br.stats_loop, (cam_pub,))]
    if gate is not None:
        threads.append((gate.run, ()))
    for fn, a in threads:
        threading.Thread(target=fn, args=a, daemon=True).start()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
