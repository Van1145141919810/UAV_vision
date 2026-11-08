# -*- coding: utf-8 -*-
"""
vb09/monitor/ros.py —— ROS 订阅 + ROS 图 / PX4 参数轮询 (start_ros)
===============================================================================
2026-10-30 第 5 步从 flight_monitor.py 原样搬来 (tools/dev/split_monitor.py)。
"""

import json
import threading
import time

from vb09.monitor.common import PX4_PARAMS, SETPOINT_PREFIX
from vb09.monitor.store import STORE


# =============================================================================
# ROS 侧
# =============================================================================
_SEL_PUB = None


def publish_select(cmd):
    """网页的选点请求 → /flight_ctrl/select (飞行脚本决定什么时候用; 见 vb09/flight/target_in.py web_select)。
    → (ok, 说明)。没连 ROS (--fake) → False。"""
    if _SEL_PUB is None:
        return False, "没连 ROS (--fake 模式?)"
    from std_msgs.msg import String
    _SEL_PUB.publish(String(data=json.dumps(cmd)))
    return True, "已发给飞行脚本"


def start_ros(args, srv=None):
    import rospy
    import rosgraph
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import State, ExtendedState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Log

    rospy.init_node("flight_monitor", anonymous=False, disable_signals=True)
    global _SEL_PUB
    _SEL_PUB = rospy.Publisher("/flight_ctrl/select", String, queue_size=5)     # 网页选点 → 飞行脚本 (2026-11-08)

    # 本节点被关 (同名节点顶掉 / master 退出 / rosnode kill) → 网页服务也退出, 让出端口。
    # 否则会留下"网页还在、ROS 已死"的僵尸: 页面显示已连接, 数据却一直不更新。
    if srv is not None:
        def _on_ros_shutdown():
            print("[monitor] ROS 节点已关闭 (被同名节点顶掉 / master 退出?) —— 网页服务一起退出")
            threading.Thread(target=srv.shutdown, daemon=True).start()
        rospy.on_shutdown(_on_ros_shutdown)

    def on_state(m):
        STORE.set_state(m.connected, m.armed, m.mode)

    def on_pose(m):
        p, q = m.pose.position, m.pose.orientation
        STORE.set_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def on_vel(m):
        v = m.twist.linear
        STORE.set_vel(v.x, v.y, v.z)

    def on_json(setter):
        def cb(m):
            try:
                setter(json.loads(m.data))
            except ValueError:
                pass
        return cb

    def on_rosout(m):
        if m.name != STORE.script_node:
            return
        msg = (m.msg or "").rstrip()
        if m.level >= Log.WARN:
            lvl = "ERROR" if m.level >= Log.ERROR else "WARN"
        elif m.level == Log.INFO:
            # 周期性状态行以空格缩进开头、分隔线以 = / - 开头 → 不进事件流
            if not msg or msg[0] in " =-" or len(msg) > 300:
                return
            lvl = "INFO"
        else:
            return
        STORE.add_event(lvl, msg, "script")

    S = rospy.Subscriber
    S("/mavros/state", State, on_state, queue_size=1)
    S("/mavros/local_position/pose", PoseStamped, on_pose, queue_size=1)
    S("/mavros/local_position/velocity_local", TwistStamped, on_vel, queue_size=1)
    S("/mavros/battery", BatteryState, lambda m: STORE.set_batt(m.voltage), queue_size=1)
    S("/mavros/extended_state", ExtendedState,
      lambda m: STORE.set_landed(m.landed_state), queue_size=1)
    S("/vins_estimator/odometry", Odometry, lambda m: STORE.tick("vins"), queue_size=1)
    S("/mavros/vision_pose/pose", PoseStamped,
      lambda m: STORE.tick("vision_pose"), queue_size=1)
    S(args.telemetry_topic, String, on_json(STORE.set_telemetry), queue_size=1)
    S(args.config_topic, String, on_json(STORE.set_config), queue_size=1)
    S("/rosout", Log, on_rosout, queue_size=50)
    # 新相机状态: 图传桥自己报 (1Hz JSON); 本程序不碰相机、不订任何图像
    S(args.cam_status_topic, String, on_json(STORE.set_cam), queue_size=1)
    # 环识别: 桥 gate / gate_gcs 模式每帧发一条 (15~30Hz)。queue_size 给大一点: 进 /api/stream 不丢帧
    S(args.gate_status_topic, String, on_json(STORE.set_gate), queue_size=10)

    def px4_loop():
        """
        PX4 机动参数 (MPC_TILTMAX_AIR 等): 只在飞控已连接且**未解锁**时读。
        读的是 mavros 自己的参数缓存 (开机后 mavros 会整表拉一遍), 不额外占 MAVLink 链路。
        """
        from mavros_msgs.srv import ParamGet
        try:
            from mavros_msgs.srv import ParamPull
        except ImportError:
            ParamPull = None
        proxy = pull = None
        while not rospy.is_shutdown():
            time.sleep(2.0)
            with STORE.lock:
                mv = dict(STORE.mavros)
                t_ok, t_try = STORE.px4.get("t"), STORE.px4.get("t_try")
            if not mv.get("connected") or mv.get("armed"):
                continue
            now = time.time()
            if t_ok is not None and now - t_ok < args.px4_refresh:
                continue
            if t_try is not None and now - t_try < 10.0 and t_ok is None:
                continue                                  # 上次失败了, 10s 后再试
            try:
                if proxy is None:
                    rospy.wait_for_service("/mavros/param/get", timeout=1.0)
                    proxy = rospy.ServiceProxy("/mavros/param/get", ParamGet)
                # ★2026-10-19 (穿环首飞问题 §3): 先确认 mavros 参数缓存就绪再逐个 get。
                #   缓存没拉完时每个 get 都失败, mavros 每次刷一行 "PR: Unknown parameter to get"。
                #   pull(force_pull=False): 缓存空就开始拉、正在拉就等它拉完、已就绪立刻返回。
                if ParamPull is not None and t_ok is None:
                    if pull is None:
                        rospy.wait_for_service("/mavros/param/pull", timeout=1.0)
                        pull = rospy.ServiceProxy("/mavros/param/pull", ParamPull)
                    STORE.set_px4(None, "参数未就绪: 等 mavros 拉完参数表 …")
                    pr = pull(force_pull=False)
                    if not pr.success or int(pr.param_received) == 0:
                        STORE.set_px4(None, "参数未就绪 (mavros 拉参数表没完成, 收到 %d 个) —— 10s 后再试"
                                      % int(pr.param_received))
                        continue
                vals, miss = {}, []
                for p in PX4_PARAMS:
                    r = proxy(param_id=p)
                    if r.success:
                        v = r.value
                        vals[p] = float(v.real) if v.real != 0.0 else int(v.integer)
                    else:
                        miss.append(p)
                STORE.set_px4(vals, ("没读到 %s (mavros 参数还没拉完?)" % ", ".join(miss))
                              if miss else None)
            except Exception as e:
                proxy = None
                STORE.set_px4(None, "读参数失败: %s" % e)

    threading.Thread(target=px4_loop, daemon=True).start()

    def graph_loop():
        master = rosgraph.Master("/flight_monitor")
        # 记住 master 的 PID。ROS1 节点在 master 重启后不会自动重新注册, 本程序的
        # 订阅会全部悄悄失效 (页面显示"无数据")。PID 一变就在事件流和机况检查里说出来。
        pid0 = None
        while not rospy.is_shutdown():
            try:
                pid = master.getPid()
                if pid0 is None:
                    pid0 = pid
                elif pid != pid0 and not STORE.graph.get("master_restarted"):
                    STORE.set_graph(master_restarted=True)
                    STORE.add_event("ERROR", "ROS master 重启过 (PID %s → %s) —— 本页数据已失效,"
                                    " 请在 tmux 里重启 flight_monitor.py" % (pid0, pid),
                                    "monitor")
                    print("[monitor] ★ ROS master 重启过, 本程序的订阅已失效, 请重启我")
                pubs, subs, _srvs = master.getSystemState()
                nodes = set()
                sp, ssub = {}, []
                img_subs, det_pubs = [], []
                for topic, ns in pubs:
                    nodes.update(ns)
                    if topic.startswith(SETPOINT_PREFIX):
                        sp[topic] = list(ns)
                    if topic == args.det_image_topic:
                        det_pubs = list(ns)
                for topic, ns in subs:
                    nodes.update(ns)
                    if topic == args.status_topic:
                        ssub = list(ns)
                    # 桥订了图像话题 = 旧版桥 (新版自己开相机, 不订图像)
                    if any("vision_bridge" in n for n in ns) and (
                            "image" in topic or topic in (args.det_image_topic, args.old_image_topic)):
                        img_subs.append(topic)
                STORE.set_graph(ok=True, nodes=sorted(nodes), setpoint_pubs=sp,
                                status_subs=ssub,
                                bridge_img_subs=sorted(img_subs), det_cam_pubs=det_pubs,
                                error=None)
            except Exception as e:
                STORE.set_graph(ok=False, error=str(e))
            STORE.watch_script()
            time.sleep(1.0)

    threading.Thread(target=graph_loop, daemon=True).start()
