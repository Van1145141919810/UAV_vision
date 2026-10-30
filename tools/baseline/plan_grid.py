# -*- coding: utf-8 -*-
"""
plan_grid.py —— 规划纯函数等价验证: plan_dash / preflight_check / static_preview 扫参数网格
  aim (红点偏/环心)、n 有/无、起点距离 2–8 m、v 1.2–2.5、D_BACK 1.5–4、虚拟环 0/2、降速策略 0/1/2
判定: 数值相同 (float 用 repr 比)。Plan 对象整个序列化 (轨迹对象按属性)。
删戳点那一步: Plan / static_preview 里戳点专用字段会消失 —— 白名单 POKE_FIELDS。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402

# 删戳点后允许消失的字段 (Plan 属性 / static_preview 键)
POKE_FIELDS = {"mode", "traj_back", "tip_gap", "pierce", "hit_v"}


def base_params(gp):
    d = dict(
        DASH_V=2.5, DASH_V_MIN=1.2, DASH_V_STEP=0.3, DASH_ACC=2.0, BRAKE_ACC=2.5, T_LAG=0.2,
        D_STRAIGHT=1.5, D_ALIGN_MIN=2.8, ENTRY_ANG_MAX=20.0, E_ALONG=0.25, V_TEAR_MIN=1.2,
        M_PASS=0.2, M_WALL=0.3, M_BACK=0.3, PASS_RUN=1.0, BRAKE_OVER_T=0.3, GATE_SLOW_POLICY=1,
        L_F=0.28, L_R=0.28, W_HALF=0.28, R_IN=0.6, AIM_MARGIN=0.10,
        D_BACK=2.0, D_REAR=2.0, D_SIDE=2.0, GATE_XY_MAX=8.0, REPO_X_MAX=-1.5,
        GATE_Z_MIN_DZ=0.5, GATE_Z_MAX_DZ=3.0, D_RUN_MAX=4.0,
        H_HALF=0.11, NEEDLE_RIGHT=0.0, NEEDLE_UP=0.0, AIM_TRIM_RIGHT=0.0, AIM_TRIM_UP=0.0)
    extra = {"GATE_POKE": False, "POKE_V": 1.0, "POKE_GAP": 0.5, "POKE_RETREAT_V": 0.4,
             "POKE_HIT_V": 0.5, "POKE_CREEP": 0.25, "POKE_DEPTH": 0.10}
    for k, v in extra.items():                     # 删戳点后 Params 不再要这些
        if k in gp.Params.FIELDS:
            d[k] = v
    return gp.Params(**d)


def run():
    gp = _compat.gate_plan()
    p0 = base_params(gp)
    origin = (0.0, 0.0, 0.0)
    c = (5.0, 0.3, 1.5)
    n0 = gp.vunit((-1.0, -0.2, 0.0))
    out = {"static": [], "plan": [], "pre": []}
    for v in (1.2, 1.8, 2.5):
        for db in (1.5, 2.5, 4.0):
            for pol in (0, 1, 2):
                q = p0.copy(DASH_V=v, D_BACK=db, GATE_SLOW_POLICY=pol)
                out["static"].append([[v, db, pol], gp.static_preview(q)])
    for aim_off in ((0.0, 0.0, 0.0), (0.0, 0.08, -0.05), (0.0, 0.35, 0.3)):
        aim = gp.vadd(c, aim_off)
        for n in (n0, None, gp.vunit((-1.0, -0.7, 0.0))):
            for dist in (2.0, 3.0, 4.0, 5.0, 6.0, 8.0):
                d_in = gp.vmul(n, -1.0) if n is not None else (1.0, 0.0, 0.0)
                p_now = gp.vsub(aim, gp.vmul(gp.hunit(d_in), dist))
                p_now = (p_now[0], p_now[1], 1.5)
                for v in (1.2, 1.8, 2.5):
                    for db in (1.5, 2.5, 4.0):
                        for vo in (0.0, 2.0):
                            for pol in (0, 1, 2):
                                if pol != 1 and (dist not in (3.0, 6.0) or vo):
                                    continue
                                q = p0.copy(DASH_V=v, D_BACK=db, GATE_SLOW_POLICY=pol)
                                key = [aim_off, n is not None and list(n), dist, v, db, vo, pol]
                                try:
                                    P = gp.plan_dash(aim, c, n, p_now, origin, q, virtual_offset=vo)
                                    out["plan"].append([key, P])
                                except Exception as e:          # 也算输出
                                    out["plan"].append([key, "EXC %r" % (e,)])
                        if dist == 4.0 and v == 2.5:
                            for vo in (0.0, 2.0):
                                try:
                                    r = gp.preflight_check(aim, c, n, origin, p0, 1.5, vo, p_now)
                                except Exception as e:
                                    r = "EXC %r" % (e,)
                                out["pre"].append([[aim_off, n is not None, vo], r])
    return out


def drop_poke(g, n):
    return _util.strip_keys(g, POKE_FIELDS), _util.strip_keys(n, POKE_FIELDS)


def main():
    write = _util.main_args()
    ok = _util.report("plan_grid", *_util.save_or_compare("plan_grid.json", run(), write,
                                                          drop=None if write else drop_poke))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
