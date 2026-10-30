# -*- coding: utf-8 -*-
"""
judge_replay.py —— 飞机脚本侧单帧判定 + 多帧融合等价验证
  一串 /gate/status (fuse_replay 的 golden 输出 + 人工改的坏帧) → GateInput.judge → gp.fuse_samples
  判定: 样本、拒绝原因、粗估结果相同
"""
import copy
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402
import synth  # noqa: E402


def _raw_statuses():
    with open(_util.golden_path("input_statuses.json")) as f:
        return json.load(f)


def mutations(st):
    """同一帧的几个坏版本, 覆盖各拒绝分支。"""
    out = []
    for k, v in (("pose", "pose_gap(120ms)"), ("sim", True), ("published", False)):
        m = copy.deepcopy(st)
        m[k] = v
        out.append(("%s=%s" % (k, v), m, 0.0))
    out.append(("old", copy.deepcopy(st), 2.0))
    m = copy.deepcopy(st)
    if m.get("ring_w") is not None:
        m["aim"] = [m["ring_w"][0], m["ring_w"][1] + 0.9, m["ring_w"][2]]
        out.append(("aim_far", m, 0.0))
    m = copy.deepcopy(st)
    if m.get("n") is not None:
        m["n_sd_deg"] = 9.0
        out.append(("n_sd", m, 0.0))
        m = copy.deepcopy(st)
        m["n"] = [0.0, 0.0, 1.0]
        out.append(("view", m, 0.0))
    m = copy.deepcopy(st)
    m.pop("ring_w", None)
    out.append(("no_ring_w", m, 0.0))
    out.append(("garbage", {"ring_w": "x"}, 0.0))
    return out


def run():
    judge, fuse, _S = _compat.judge()
    rig = synth.make_rig()
    seq = synth.sequence(rig)
    sts = _raw_statuses()
    res, win, fused = [], [], []
    for i, (st, (t, T_WC, p, q, fl)) in enumerate(zip(sts, seq)):
        p_now = tuple(float(v) for v in p)
        now = float(st.get("t", t)) + 0.3
        smp, why = judge(st, now, p_now)
        res.append([why, smp])
        if smp is not None:
            win.append(smp)
            win = win[-12:]
            d = None
            if smp.get("n") is not None:
                d = tuple(-v for v in smp["n"])
            fused.append(fuse(win, p_now, d))
        if i % 6 == 0:
            for name, m, dt in mutations(st):
                res.append(["mut:" + name, list(judge(m, now + dt, p_now))])
    # 没位姿
    res.append(["no_pose", list(judge(sts[30], float(sts[30]["t"]) + 0.3, None))])
    return {"judge": res, "fused": fused}


def main():
    write = _util.main_args()
    ok = _util.report("judge", *_util.save_or_compare("judge.json", run(), write))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
