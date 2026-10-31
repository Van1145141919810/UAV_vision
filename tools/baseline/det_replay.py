# -*- coding: utf-8 -*-
"""
det_replay.py —— 检测库等价验证
  地面路径: GateDetector.process(L, R, t, None) → frame_to_msg      (t_ms 耗时字段除外)
  机上路径: GateDetector.process(L, R, t, T_WC) → target / stage / used (vision_bridge gate 模式)
输入: 合成序列 (synth.py); 给 --rec DIR 时另跑 gcs_gate --record 的录像 (TODO: 等录像)。
  python3 det_replay.py --write   存 golden (只在存档版本上跑)
  python3 det_replay.py           和 golden 比
副产品: golden/det_msgs.json 也是 fuse_replay 的输入。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402
import synth  # noqa: E402


def run():
    gd, wire = _compat.det(), _compat.wire()
    rig = synth.make_rig()
    seq = synth.sequence(rig)
    det_g = gd.GateDetector(rig, gd.GateConfig())       # 地面: 不给位姿
    det_a = gd.GateDetector(rig, gd.GateConfig())       # 机上: 给位姿
    msgs, onboard = [], []
    for i, (t, T_WC, p, q, fl) in enumerate(seq):
        L, R = synth.render_pair(rig, T_WC, fl["dot"], fl["visible"], seed=i)
        fr = det_g.process(L, R, t, None)
        m = wire.frame_to_msg(fr)
        m.pop("t_ms", None)
        msgs.append(m)
        fa = det_a.process(L, R, t, T_WC)
        tg = fa.target
        onboard.append({"stage": fa.stage, "used": fa.used, "rejected": fa.rejected,
                        "skipped_ring": fa.skipped_ring, "dot_roi": fa.dot_roi,
                        "target": {k: getattr(tg, k, None) for k in
                                   ("valid", "pos_w", "cov_w", "src", "age", "dist", "side")},
                        "msg": _util.strip_keys(wire.frame_to_msg(fa), {"t_ms"})})
    rig_msg = wire.rig_to_msg(rig)
    # ★第 6 步 3: rig 多了 lib (检测库指纹, 每次改代码都变) / cfg —— 不进基线 (fuse_replay 现算现填)
    rig_msg = {k: v for k, v in rig_msg.items() if k not in ("lib", "cfg")}
    return {"rig": rig_msg, "msgs": msgs}, onboard


def main():
    write = _util.main_args()
    ground, onboard = run()
    if write:                                            # fuse_replay 的输入 (原始 JSON, float 精确往返)
        import json
        with open(_util.golden_path("input_det_msgs.json"), "w") as f:
            json.dump(ground, f)
    ok1 = _util.report("det_ground", *_util.save_or_compare("det_msgs.json", ground, write))
    ok2 = _util.report("det_onboard", *_util.save_or_compare("det_onboard.json", onboard, write))
    return 0 if ok1 and ok2 else 1


if __name__ == "__main__":
    sys.exit(main())
