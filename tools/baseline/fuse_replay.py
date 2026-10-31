# -*- coding: utf-8 -*-
"""
fuse_replay.py —— 飞机侧融合等价验证 (gate_gcs 模式)
  同一批地面观测 (golden/input_det_msgs.json) + 位姿序列 (synth) →
  vision_bridge.GateFuseWorker.on_obs (PoseBuffer → GateFuser → _ring_extra → GateOut.emit)
  判定: /gate/status JSON 逐字段相同 (ms / det_ms 耗时字段除外)
另跑一遍"地面 sim=True"和"位姿缺失"的变体, 覆盖 ground_sim / pose 分支。
★第 6 步 3 (版本检查): 基线里的 rig 不带 lib / cfg → 这里按本机现算现填 (= 两边一致, 输出和以前相同);
  另有 ver_* 变体 (不进基线, 现场断言): 地面缺版本 / 检测库不同 / GateConfig 不同 / 消息版本不同
  / 目标类型不同 (ver_kind) → 每条 /gate/status 都 sim = true 且带 ver_bad。
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402
import synth  # noqa: E402

TIMING = {"ms", "det_ms"}


def _cfg():
    c = {"gate_params": "/nonexistent/gate_params.json", "max_pose_gap": 0.1, "gate_sim": False,
         "gate_reset_age": 0.0}
    c.update(synth.CAM)
    return c


def run(inp, variant):
    vb = _compat.bridge()
    import rospy
    from geometry_msgs.msg import PoseStamped
    out = []
    rospy._bus.pop("/gate/status", None)
    rospy.Subscriber("/gate/status", None, lambda m: out.append(json.loads(m.data)))
    w = vb.GateFuseWorker(_cfg())
    from vb09.common import wire
    rig_in = dict(inp["rig"], lib=wire.lib_version(), cfg=wire.cfg_digest(w.gcfg))
    if variant == "ver_old_ground":
        rig_in = dict(inp["rig"])
    elif variant == "ver_lib":
        rig_in["lib"] = "0" * 12
    elif variant == "ver_cfg":
        rig_in["cfg"] = "0" * 12
    elif variant == "ver_kind":                  # 接线: 地面跑的是别的目标类型
        rig_in["kind"] = "reddot"
    rig = synth.make_rig()
    seq = synth.sequence(rig)
    used = []
    for i, ((t, T_WC, p, q, fl), m) in enumerate(zip(seq, inp["msgs"])):
        skip_pose = variant == "pose_gap" and 20 <= i < 24
        if not skip_pose:
            ps = PoseStamped()
            ps.header.stamp = rospy.Time(t)
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in p)
            (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
             ps.pose.orientation.w) = q
            w.on_pose(ps)
        g = json.loads(json.dumps(m))
        if variant == "ver_msg":
            g["v"] = 999
        det = {"gate": g, "rig": rig_in, "t_cap": t, "sim": variant == "ground_sim"}
        used.append(w.on_obs(det))
    bad = w.on_obs({"gate": None, "rig": rig_in, "t_cap": 0.0})       # 坏包
    return {"status": _util.strip_keys(out, TIMING), "used": used, "bad_pkt": bad, "n_reset": w.n_reset}


def main():
    write = _util.main_args()
    with open(_util.golden_path("input_det_msgs.json")) as f:
        inp = json.load(f)
    ok = True
    for v in ("normal", "ground_sim", "pose_gap"):
        r = run(inp, v)
        if write and v == "normal":                      # judge_replay 的输入 (原始 JSON)
            with open(_util.golden_path("input_statuses.json"), "w") as f:
                json.dump(r["status"], f)
        ok &= _util.report("fuse_" + v, *_util.save_or_compare("fuse_%s.json" % v, r, write))
    for v, want in (("ver_old_ground", "旧版"), ("ver_lib", "检测库"), ("ver_cfg", "GateConfig"),
                    ("ver_msg", "消息版本"), ("ver_kind", "目标类型")):
        st = run(inp, v)["status"]
        why = [s for s in st if not (s.get("sim") is True and s.get("ver_bad")
                                     and any(want in x for x in s["ver_bad"]))]
        ok &= _util.report("fuse_" + v, not why and len(st) > 0,
                           "%d/%d 条 sim=true + ver_bad(%s)" % (len(st) - len(why), len(st), want))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
