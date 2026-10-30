# -*- coding: utf-8 -*-
"""
gate_plan.py —— 穿环冲刺的规划 (纯函数, 不 import ROS / numpy)
===============================================================================
stage12_gate.py 用它; 也能在笔记本上离线跑:  python3 gate_plan.py   (打印一组例子)

坐标约定 (和 INTERFACE_gate.md 第 0 节、flight_monitor.gate_axes 同一套)
-------------------------------------------------------------------------------
  world  = VIO odom (ENU, 米)
  n      = 环平面单位法向, **指向进近侧** (飞机过来的那一边)
  靶面   = 站在进近侧看向环: d = −n, right = normalize(d × z), up = right × d
           h = (p − c)·right, v = (p − c)·up
  along  = (p − c)·n          离环越近越小, 穿过环平面 = 0, 过环后为负

冲刺坐标 s (本文件内部用)
  原点 = 瞄点 aim, 沿进入方向 dir (水平单位向量, 朝环飞为正):
  s = (p − aim)·dir。纸面 s = 0, 过环后 s > 0。dir = −n 时 s = −along。

方案 v1 (PLAN_gate_v1.md) 的规划部分, 2026-10-19 讨论后的两处修正:
  1. 轨迹从 s_b = s_stop − v²/(2·BRAKE_ACC) 开始刹车。**控制滞后 T_LAG 不进轨迹**:
     飞机跟踪一条按时间参数化的速度曲线时, 滞后只让它"晚到", 不改变停在哪里。
     T_LAG 只用来估计撞纸速度 (飞机落后于轨迹, 实际减速比轨迹早一点)。
  2. 助跑距离有下限 D_ALIGN_MIN: 起跑点要能看到整个环 (≥ 2.4m), 才能在那里锁定估计。
  3. 起跑点动态 (start_options): 飞机离环在 [最小助跑, D_RUN_MAX] 之间就地起跑,
     近了后退、远了前进。手动段把飞机开到哪, 就从哪起跑。

★2026-10-21 刹车点改成"过纸后再冲 PASS_RUN"(dash_stop), 取代旧的"停止窗口居中":
  旧: s_stop = 窗口中点, s_b = s_stop − v²/2a → 2.5 m/s 时机头一碰纸就开始减速, 纸容易缠在机身上。
  新: s_b    = −L_F + PASS_RUN + v·T_LAG     机头碰纸后**飞机实际**还匀速冲 PASS_RUN 才开始减速
      s_stop = s_b + v²/(2·BRAKE_ACC)
      可行:    s_stop + L_F + M_BACK ≤ D_BACK  刹停后机头离环后墙至少 M_BACK
  v·T_LAG 是滞后补偿: 匀速段飞机落后轨迹 v·T_LAG, 轨迹开始减速时飞机还没到 s_b。
  补偿不影响最终停在哪 (飞机最后会追上轨迹), 只是多占环后空间。
  s_stop 随速度变, 放不下就按 DASH_V_STEP 降速 (和其它场地限制一样)。
  L_F / L_R 在冲刺档案里只是机身 (含护桨, 不含针)。M_PASS / E_ALONG 不再决定刹车点。

★2026-10-21(2) 两处修正 (flight_0009 实测):
  · 刹车超调: 规划停在 s=1.82, 实际停在 2.42 (+0.59m @ 2.0m/s)。撞纸掉速后 PX4 追位置把速度冲过头,
    刹车时倾角顶到 MPC_TILTMAX_AIR。离墙 / 水平范围检查改用 s_end = s_stop + BRAKE_OVER_T·v
    (BRAKE_OVER_T 默认 0.3s)。轨迹本身不变, 只是查空间时多算这段。
  · 不许静默降速: GATE_SLOW_POLICY (档案 allow_slow)
      0 否   按 DASH_V 不可行 → 规划失败, 说明卡在哪、场地最高能冲多快
      1 确认 (默认) 降到能飞的速度, 但冲刺前必须按 Y 确认 (G 不算)
      2 是   自动降速 (照样大声告警)
    preflight_check(): 起飞前用地面看到的环位置先算一遍, 不用飞起来才发现。
===============================================================================
"""

import math

# =============================================================================
# 小向量工具 (3 元组)
# =============================================================================


def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vmul(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def vnorm(a):
    return math.sqrt(vdot(a, a))


def vunit(a):
    n = vnorm(a)
    return vmul(a, 1.0 / n) if n > 1e-12 else (0.0, 0.0, 0.0)


def hunit(a):
    """水平分量单位化 (z 置 0)。太短返回 None。"""
    h = (a[0], a[1], 0.0)
    n = vnorm(h)
    return vmul(h, 1.0 / n) if n > 1e-9 else None


def ang_deg(a, b):
    c = max(-1.0, min(1.0, vdot(vunit(a), vunit(b))))
    return math.degrees(math.acos(c))


def gate_axes(n):
    """靶面坐标轴 (right, up)。和 flight_monitor.gate_axes 逐字同一公式。"""
    d = vmul(n, -1.0)
    right = vunit(vcross(d, (0.0, 0.0, 1.0)))
    return right, vcross(right, d)


def to_plane(p, c, n):
    """→ (h, v, along)。见文件头。"""
    r, u = gate_axes(n)
    q = vsub(p, c)
    return vdot(q, r), vdot(q, u), vdot(q, n)


def yaw_of(d):
    return math.atan2(d[1], d[0])


def entry_axes(d):
    """进入方向 d (朝环飞为正) → (right, up, d)。和 gate_axes(n = −d) 同一套靶面轴。"""
    right, up = gate_axes(vmul(d, -1.0))
    return right, up, d


def sigma_hva(d, cov=None, sd_cm=None):
    """
    环心位置 σ 投到靶面: → (σ_h, σ_v, σ_along) cm (首飞问题 §1)。
      cov    9 个数 (m², 行主序 3×3, world) —— /gate/status 的 cov
      sd_cm  3 个数 (cm, world xyz) —— 没有 cov 时当对角协方差用
    都没有 → None
    """
    if cov is not None and len(cov) >= 9:
        C = [[float(cov[3 * i + j]) * 1e4 for j in range(3)] for i in range(3)]
    elif sd_cm is not None and len(sd_cm) >= 3:
        C = [[float(sd_cm[i]) ** 2 if i == j else 0.0 for j in range(3)] for i in range(3)]
    else:
        return None
    out = []
    for a in entry_axes(d):
        q = sum(a[i] * C[i][j] * a[j] for i in range(3) for j in range(3))
        out.append(math.sqrt(max(q, 0.0)))
    return tuple(out)


def spread_hva(points, d):
    """一组世界点按靶面轴的 p90−p10 → (h, v, along) cm。"""
    if len(points) < 2:
        return (0.0, 0.0, 0.0)
    return tuple(pct_range([vdot(q, a) for q in points]) * 100.0 for a in entry_axes(d))


# =============================================================================
# 参数
# =============================================================================
class Params(object):
    """规划用到的全部参数。stage12 从自己的全局里拼出来 (见 params_from_globals)。"""
    FIELDS = (
        "DASH_V", "DASH_V_MIN", "DASH_V_STEP", "DASH_ACC", "BRAKE_ACC", "T_LAG",
        "D_STRAIGHT", "D_ALIGN_MIN", "ENTRY_ANG_MAX", "E_ALONG", "V_TEAR_MIN",
        "M_PASS", "M_WALL", "M_BACK", "PASS_RUN", "BRAKE_OVER_T", "GATE_SLOW_POLICY", "L_F", "L_R", "W_HALF", "R_IN", "AIM_MARGIN",
        "D_BACK", "D_REAR", "D_SIDE", "GATE_XY_MAX", "REPO_X_MAX",
        "GATE_Z_MIN_DZ", "GATE_Z_MAX_DZ", "D_RUN_MAX",
        "H_HALF", "NEEDLE_RIGHT", "NEEDLE_UP", "AIM_TRIM_RIGHT", "AIM_TRIM_UP",
        # ★2026-10-20 戳点模式 (针戳红点, 不穿环): L_F 此时 = 机体中心 → 针尖
        "GATE_POKE", "POKE_V", "POKE_GAP", "POKE_RETREAT_V", "POKE_HIT_V", "POKE_CREEP", "POKE_DEPTH",
    )

    def __init__(self, **kw):
        for k in self.FIELDS:
            setattr(self, k, kw.get(k))
        miss = [k for k in self.FIELDS if getattr(self, k) is None]
        if miss:
            raise ValueError("gate_plan.Params 缺参数: %s" % ", ".join(miss))

    def copy(self, **kw):
        d = {k: getattr(self, k) for k in self.FIELDS}
        d.update(kw)
        return Params(**d)


def params_from_globals(g):
    return Params(**{k: g[k] for k in Params.FIELDS if k in g})


# =============================================================================
# 刹车点 —— 过纸后再冲 PASS_RUN (2026-10-21, 见文件头)
# =============================================================================
def dash_stop(v, p):
    """
    速度 v 时的刹车点。→ dict
      s_b     轨迹刹车起点 = −L_F + PASS_RUN + v·T_LAG
      s_stop  停止点 (机体中心) = s_b + v²/(2·BRAKE_ACC)
      s_hi    停止点上限 = D_BACK − L_F − M_BACK (机头离环后墙 M_BACK)
      over    刹车超调估计 BRAKE_OVER_T·v (实测: 撞纸后 PX4 追位置 + 刹车倾角饱和)
      s_end   飞机实际大约停在哪 = s_stop + over  (离墙 / 水平范围都按它查)
      margin  s_hi − s_end: 比要求的 M_BACK 还多出来的空间, < 0 = 放不下
      lag     滞后补偿 v·T_LAG
      d_back_need  这个速度需要的 D_BACK
    """
    lag = v * p.T_LAG
    s_b = -p.L_F + p.PASS_RUN + lag
    s_stop = s_b + v * v / (2.0 * p.BRAKE_ACC)
    over = p.BRAKE_OVER_T * v
    s_end = s_stop + over
    s_hi = p.D_BACK - p.L_F - p.M_BACK
    return {"s_b": s_b, "s_stop": s_stop, "over": over, "s_end": s_end, "s_hi": s_hi,
            "margin": s_hi - s_end, "lag": lag, "d_back_need": s_end + p.L_F + p.M_BACK}


def dash_v_room(p, d_back=None):
    """
    场地 (D_BACK) 允许的最高冲刺速度: 从 DASH_V 按 DASH_V_STEP 往下试到 DASH_V_MIN
    (和 plan_dash 的降速同一个序列)。都放不下 → None。只看环后空间, 不看其它场地限制。
    """
    q = p if d_back is None else p.copy(D_BACK=d_back)
    v = q.DASH_V
    while v >= q.DASH_V_MIN - 1e-9:
        if dash_stop(v, q)["margin"] >= -1e-9:
            return v
        v = round(v - q.DASH_V_STEP, 6)
    return None


def pass_short(p):
    """PASS_RUN 比整个机身 (L_F + L_R) 还短 → 开始减速时机尾还在纸里。→ 说明文字或 None"""
    body = p.L_F + p.L_R
    if p.PASS_RUN < body + 0.1 - 1e-9:
        return ("过纸冲刺 %.2fm 不比机身长 %.2fm 多出 10cm: 开始减速时机尾还在纸里或刚出纸, 纸可能缠上机身"
                % (p.PASS_RUN, body))
    return None


def d_run_for(v, p):
    """助跑距离: 加速段 + 稳定直线段, 但不小于 D_ALIGN_MIN (起跑点要看得到整个环)。
    戳点模式 (GATE_POKE): 按轨迹实际需要的长度算, 见 poke_d_min。"""
    if getattr(p, "GATE_POKE", False):
        return poke_d_min(v, p)
    return max(v * v / (2.0 * p.DASH_ACC) + p.D_STRAIGHT, p.D_ALIGN_MIN)


def poke_pierce(p):
    """戳破 / 顶开模式: 有接触速度 (POKE_HIT_V>0) 且 POKE_GAP ≤ 0。否则是"停在纸前"的老戳点。"""
    return p.POKE_HIT_V > 1e-9 and p.POKE_GAP <= 1e-9


def poke_hit_v(p, v=None):
    """实际接触速度: 不高于巡航速度。"""
    return min(p.POKE_HIT_V, p.POKE_V if v is None else v)


def poke_d_min(v, p):
    """戳点最小助跑 (机体中心离纸面): 加速 + 减速 (+ 爬行) 放得下, 且不小于 D_ALIGN_MIN。"""
    acc_d = v * v / (2.0 * p.DASH_ACC)
    if poke_pierce(p):
        h = poke_hit_v(p, v)
        need = acc_d + (v * v - h * h) / (2.0 * p.BRAKE_ACC) + p.POKE_CREEP + p.L_F + 0.1
    else:
        need = acc_d + v * v / (2.0 * p.BRAKE_ACC) + p.L_F + max(p.POKE_GAP, 0.0) + 0.1
    return max(need, p.D_ALIGN_MIN)


def r_safe(p):
    """瞄点离环心最多多远 (飞机半宽 + 10cm 余量)。"""
    return p.R_IN - p.W_HALF - p.AIM_MARGIN


# =============================================================================
# 一维轨迹: 加速 → 匀速 → 刹车, 按时间采样
# =============================================================================
class Traj1D(object):
    """
    s 从 s0 (= −d_run) 出发, 以 acc 加速到 v, 匀速到 s_b, 以 dec 减到 0, 止于 s_stop。
    at(t) → (s, ds/dt, d²s/dt²), t 从 0 开始; t < 0 停在 s0, t > T 停在 s_stop。
    """

    def __init__(self, s0, s_stop, v, acc, dec):
        self.s0, self.s_stop, self.v, self.acc, self.dec = s0, s_stop, v, acc, dec
        self.t1 = v / acc
        self.s1 = s0 + v * v / (2.0 * acc)
        self.s_b = s_stop - v * v / (2.0 * dec)
        if self.s_b < self.s1 - 1e-9:
            raise ValueError("轨迹放不下: 加速段终点 %.2f 已经过了刹车起点 %.2f" % (self.s1, self.s_b))
        self.t2 = self.t1 + (self.s_b - self.s1) / v
        self.T = self.t2 + v / dec

    def at(self, t):
        if t <= 0.0:
            return self.s0, 0.0, 0.0
        if t < self.t1:
            return self.s0 + 0.5 * self.acc * t * t, self.acc * t, self.acc
        if t < self.t2:
            return self.s1 + self.v * (t - self.t1), self.v, 0.0
        if t < self.T:
            tb = t - self.t2
            return self.s_b + self.v * tb - 0.5 * self.dec * tb * tb, self.v - self.dec * tb, -self.dec
        return self.s_stop, 0.0, 0.0

    def t_of_s(self, s):
        """轨迹第一次到达 s 的时刻 (s 超出范围时夹到两端)。"""
        if s <= self.s0:
            return 0.0
        if s < self.s1:
            return math.sqrt(2.0 * (s - self.s0) / self.acc)
        if s < self.s_b:
            return self.t1 + (s - self.s1) / self.v
        if s < self.s_stop:
            # s = s_b + v·tb − dec·tb²/2  →  取小根
            disc = max(0.0, self.v * self.v - 2.0 * self.dec * (s - self.s_b))
            return self.t2 + (self.v - math.sqrt(disc)) / self.dec
        return self.T

    def v_at_s(self, s):
        return self.at(self.t_of_s(s))[1]


class TrajPoke(object):
    """
    戳破 / 顶开用的五段轨迹 (s 是机体中心沿进入方向的坐标, 纸面 s=0):
      加速 (acc) → 巡航 v → 减速 (dec) 到接触速度 h → 以 h 匀速爬行 → 过接触点 s_c 后继续压入 → 刹停在 s_end。
    爬行段从 s_c0 = s_c − creep 开始: 纸的实际位置有 ±10cm 的误差, 爬行段让接触速度恒为 h, 不受误差影响。
    字段名沿用 Traj1D: s0 / s_stop / v / T / t1 / t2 (t2 = 爬行结束 / 开始最后刹车) / s_b (最后刹车起点)。
    """

    def __init__(self, s0, s_c, depth, v, h, acc, dec, creep):
        h = min(h, v)
        self.s0, self.v, self.h, self.acc, self.dec = s0, v, h, acc, dec
        self.s_c = s_c
        self.s_stop = s_c + depth
        self.s_c0 = s_c - creep
        self.s_b = self.s_stop - h * h / (2.0 * dec)       # 最后刹车起点 (从 h 刹到 0)
        if self.s_b < s_c - 1e-9:
            raise ValueError("压入深度 %.0fcm 小于从接触速度 %.2fm/s 刹停所需 %.0fcm: 加大 POKE_DEPTH 或降低 POKE_HIT_V"
                             % (depth * 100, h, h * h / (2.0 * dec) * 100))
        self.t1 = v / acc
        self.s1 = s0 + v * v / (2.0 * acc)                  # 加速完
        self.s_d0 = self.s_c0 - (v * v - h * h) / (2.0 * dec)   # 开始减速到 h
        if self.s_d0 < self.s1 - 1e-9:
            raise ValueError("轨迹放不下: 加速段终点 %.2f 已经过了减速起点 %.2f (加大助跑 / 降低 POKE_V / 缩短 POKE_CREEP)"
                             % (self.s1, self.s_d0))
        self.t_cruise = (self.s_d0 - self.s1) / v
        self.t_d = (v - h) / dec
        self.t_creep = (self.s_b - self.s_c0) / h if h > 1e-9 else 0.0
        self.ta = self.t1
        self.tb = self.ta + self.t_cruise
        self.tc = self.tb + self.t_d
        self.t2 = self.tc + self.t_creep
        self.T = self.t2 + h / dec

    def at(self, t):
        if t <= 0.0:
            return self.s0, 0.0, 0.0
        if t < self.ta:
            return self.s0 + 0.5 * self.acc * t * t, self.acc * t, self.acc
        if t < self.tb:
            return self.s1 + self.v * (t - self.ta), self.v, 0.0
        if t < self.tc:
            u = t - self.tb
            return self.s_d0 + self.v * u - 0.5 * self.dec * u * u, self.v - self.dec * u, -self.dec
        if t < self.t2:
            return self.s_c0 + self.h * (t - self.tc), self.h, 0.0
        if t < self.T:
            u = t - self.t2
            return self.s_b + self.h * u - 0.5 * self.dec * u * u, self.h - self.dec * u, -self.dec
        return self.s_stop, 0.0, 0.0


def make_poke_traj(d_run, v, p):
    """按参数造去程轨迹: 戳破 → TrajPoke; 停在纸前 → Traj1D。"""
    if poke_pierce(p):
        return TrajPoke(-d_run, -p.L_F, p.POKE_DEPTH, v, poke_hit_v(p, v), p.DASH_ACC, p.BRAKE_ACC, p.POKE_CREEP)
    return Traj1D(-d_run, poke_end_s(p), v, p.DASH_ACC, p.BRAKE_ACC)


class Mirror(object):
    """
    把 Traj1D 在 s 轴上翻转 (s → −s): 戳完退回用。内部轨迹 u 从 u0 走到 u1 (u = −s),
    对外 at(t) 给的是 s 的 (位置, 速度, 加速度)。T / t2 / s_b 等沿用内部轨迹的字段名。
    """

    def __init__(self, tr):
        self.tr = tr
        self.T, self.t1, self.t2, self.v = tr.T, tr.t1, tr.t2, tr.v

    @property
    def s0(self):
        return -self.tr.s0

    @property
    def s_stop(self):
        return -self.tr.s_stop

    def at(self, t):
        s, v, a = self.tr.at(t)
        return -s, -v, -a


def simulate_lag(traj, tau, dt=0.002):
    """
    飞机 = 速度一阶滞后跟踪轨迹速度 (时间常数 tau)。只用来估计撞纸速度和停止点,
    是偏保守的模型 (P 模式有位置反馈, 实际跟得更紧)。
    → [(t, s, v)] 采样表
    """
    out = []
    s, v, t = traj.s0, 0.0, 0.0
    t_end = traj.T + 6.0 * max(tau, 0.05)
    while t <= t_end:
        vr = traj.at(t)[1]
        v += (vr - v) * (dt / tau if tau > 1e-6 else 1.0)
        s += v * dt
        t += dt
        out.append((t, s, v))
    return out


def v_at_vehicle_s(samples, s_query):
    for _, s, v in samples:
        if s >= s_query:
            return v
    return 0.0


# =============================================================================
# 进入方向
# =============================================================================
def entry_dir(aim, n, p_now, ang_max_deg):
    """
    → (dir, 说明)。水平单位向量, 朝环飞为正。
      有法向: 在 −n 和"飞机→瞄点视线"之间, 取与 −n 夹角 ≤ ang_max 且最接近视线的方向。
      没法向: 就是视线方向 (起飞点要放在环的正前方, 斜放的环由操作员负责)。
    """
    los = hunit(vsub(aim, p_now))
    if n is None:
        return los, "视线(无法向)"
    nh = hunit(vmul(n, -1.0))
    if nh is None:
        return los, "视线(法向竖直?)"
    if los is None:
        return nh, "−n"
    a = ang_deg(nh, los)
    if a <= ang_max_deg:
        return los, "视线(与 −n 夹 %.0f°)" % a
    # 从 nh 往 los 转 ang_max (水平面内)
    sgn = 1.0 if vcross(nh, los)[2] >= 0 else -1.0
    th = math.radians(ang_max_deg) * sgn
    c, s = math.cos(th), math.sin(th)
    d = (nh[0] * c - nh[1] * s, nh[0] * s + nh[1] * c, 0.0)
    return d, "−n 转 %.0f° (视线偏 %.0f°)" % (ang_max_deg, a)


def body_fits(h, v, p):
    """
    机体中心在靶面 (h, v) 处时, 机身截面 (宽 2·W_HALF × 高 2·H_HALF 的矩形) 四个角
    都在内圈里、并留 AIM_MARGIN。机身扁 (高 22cm), 上下比左右能偏得多。
    """
    r = p.R_IN - p.AIM_MARGIN
    return (abs(h) + p.W_HALF) ** 2 + (abs(v) + p.H_HALF) ** 2 <= r * r + 1e-12


def clamp_aim(aim, c, n, p):
    """
    机体中心瞄点 aim 限制在"机身穿得过去"的区域里 (body_fits)。放不下就沿环心方向往回拉。
    n 是靶面法向 (没有真法向时传 −进入方向)。→ (aim', 是否被限)。
    """
    if c is None or n is None:
        return aim, False
    h, v, along = to_plane(aim, c, n)
    if body_fits(h, v, p):
        return aim, False
    lo, hi = 0.0, 1.0                      # 二分: 最大的 k 使 (k·h, k·v) 放得下
    if not body_fits(0.0, 0.0, p):
        hi = 0.0
    for _ in range(30):
        k = 0.5 * (lo + hi)
        if body_fits(k * h, k * v, p):
            lo = k
        else:
            hi = k
    right, up = gate_axes(n)
    q = vadd(vadd(c, vmul(right, h * lo)), vmul(up, v * lo))
    return vadd(q, vmul(n, along)), True        # 沿法向的分量 (纸面深度) 保持不变


# =============================================================================
# 规划
# =============================================================================
class Plan(object):
    """一次规划的结果。ok=False 时 errors 里是不可行的原因。"""

    def __init__(self):
        self.ok = False
        self.errors, self.warnings, self.notes = [], [], []
        self.v = None
        self.dir = None
        self.dir_why = ""
        self.aim = None              # 机体中心的瞄点 (轨迹沿这条线飞)
        self.aim_tool = None         # 想让针尖打到的点 (红点 / 环心)
        self.tool_miss = 0.0         # 被安全区拉回后, 针尖会偏离 aim_tool 多远 (m)
        self.aim_clamped = False
        self.S = self.E = None
        self.d_run = None
        self.d_run_min = None        # 这个速度的最小助跑
        self.start_how = ""          # 就地起跑 / 后退 / 前进 (start_options)
        self.s_lo = self.s_hi = self.s_stop = self.tol = None
        #   冲刺 (2026-10-21): s_lo = s_stop, s_hi = 停止点上限, tol = 离环后墙的富余 (s_hi − s_stop)
        self.pass_lag = None         # 冲刺: 滞后补偿 v·T_LAG
        self.pass_actual = None      # 冲刺: 滞后模型下飞机开始减速时机头过纸多远 (≈ PASS_RUN)
        self.s_end = None            # 冲刺: 含刹车超调的实际停止点估计
        self.E_end = None            # 冲刺: s_end 对应的世界点 (水平范围按它查)
        self.v_set = None            # 冲刺: 设定的 DASH_V
        self.v_room = None           # 冲刺: 场地最高能冲多快 (不可行时也算出来, 给提示用)
        self.slow_why = ""           # 冲刺: 降速 / 不可行的原因 (按 DASH_V 时卡在哪)
        self.slow = False            # 冲刺: 计划速度 < DASH_V
        self.need_confirm = False    # 冲刺: 降速了且 GATE_SLOW_POLICY = 1 → 冲刺前要按 Y
        self.brake_dist = None      # 含滞后的"刹车距离" (显示用)
        self.v_hit_nom = self.v_hit_worst = None
        self.veh_stop = None        # 滞后模型下飞机实际停在哪 (s)
        self.traj = None
        self.speed_chain = []
        self.mode = "dash"           # "dash" 冲刺穿环 / "poke" 针戳红点后原路退回
        self.traj_back = None        # poke: 退回起跑点的轨迹 (Mirror, s 从 s_stop 回到 −d_run)
        self.tip_gap = None        # poke: 到终点时针尖离纸面多远 (负 = 扎进去)
        self.pierce = False        # poke: 戳破 / 顶开模式
        self.hit_v = None          # poke 戳破: 接触速度

    def summary(self):
        if not self.ok:
            return "不可行: " + "; ".join(self.errors)
        if self.mode == "poke" and self.pierce:
            return ("戳破 v=%.2fm/s 助跑 %.2fm [%s] 接触 %.2fm/s 压入 %.0fcm 终点 s=%+.2f 去程 %.1fs 退回 %.1fs  方向 %s"
                    % (self.v, self.d_run, self.start_how, self.hit_v,
                       (self.s_stop - self.traj.s_c) * 100, self.s_stop, self.traj.T, self.traj_back.T,
                       self.dir_why))
        if self.mode == "poke":
            return ("戳点 v=%.2fm/s 助跑 %.2fm [%s] 终点 s=%+.2f (针尖%s纸面 %.0fcm) "
                    "接触速度 %.2f/最坏 %.2f m/s 去程 %.1fs 退回 %.1fs  方向 %s"
                    % (self.v, self.d_run, self.start_how, self.s_stop,
                       "离" if self.tip_gap >= 0 else "扎进", abs(self.tip_gap) * 100,
                       self.v_hit_nom, self.v_hit_worst, self.traj.T, self.traj_back.T,
                       self.dir_why))
        return ("v=%.2fm/s 助跑 %.2fm [%s] 过纸冲刺 %.2fm (补偿 %.2f) 刹车起点 s=%+.2f 停止 s=%+.2f "
                "(+超调 %.2f, 离墙再富余 %.2fm) 撞纸 %.2f m/s 用时 %.2fs  方向 %s"
                % (self.v, self.d_run, self.start_how, self.pass_actual, self.pass_lag,
                   self.traj.s_b, self.s_stop, self.s_end - self.s_stop, self.tol, self.v_hit_nom,
                   self.traj.T, self.dir_why))

    def point(self, s):
        """冲刺直线上 s 处的世界点。"""
        return vadd(self.aim, vmul(self.dir, s))


def _ring_frame(dir_, n):
    """场地检查用的环系: x 沿 −n (没有法向时沿 dir), y = 水平向右。"""
    fwd = hunit(vmul(n, -1.0)) if n is not None else dir_
    if fwd is None:
        fwd = dir_
    right = vunit(vcross(fwd, (0.0, 0.0, 1.0)))
    return fwd, right


def start_options(dist_now, v, p):
    """
    动态起跑点 (2026-10-19): 助跑距离 d_run 的候选, 按优先级。→ [(d_run, 说明)]
      飞机离瞄点 dist_now 在 [d_min, D_RUN_MAX] 里 → 就地起跑 (不摆位)
      比 d_min 近 → 后退到 d_min;  比 D_RUN_MAX 远 → 前进到 D_RUN_MAX
    就地 / 前进的候选放不下 (后方空地、侧向余量…) 时, 再试 d_min。D_RUN_MAX ≤ d_min = 老行为。
    """
    d_min = d_run_for(v, p)
    d_cap = max(d_min, p.D_RUN_MAX)
    if dist_now < d_min:
        return [(d_min, "后退到最小助跑 %.2fm" % d_min)]
    if dist_now <= d_cap:
        out = [(dist_now, "就地起跑 (助跑 %.2fm)" % dist_now)]
    else:
        out = [(d_cap, "前进到 D_RUN_MAX %.2fm" % d_cap)]
    if out[0][0] > d_min + 1e-6:
        out.append((d_min, "最小助跑 %.2fm" % d_min))
    return out


def _try_start(P, aim, d, fwd, right, x_origin, x_rear_lim, y_lim, p_now, origin, v, d_run, pv):
    """按速度 v、助跑 d_run 试一次。可行 → 填进 P 并返回 []; 否则返回原因列表。"""
    errs = []
    S = vadd(aim, vmul(d, -d_run))
    E = vadd(aim, vmul(d, P.s_stop))
    E_end = vadd(aim, vmul(d, P.s_end))          # 含刹车超调: 场地检查按它
    try:
        tr = Traj1D(-d_run, P.s_stop, v, pv.DASH_ACC, pv.BRAKE_ACC)
    except ValueError as e:
        errs.append(str(e))
        tr = None
    # ---- 场地 (环系)
    for nm, q in (("起跑点 S", S), ("制动终点 E (含超调)", E_end)):
        xq, yq = vdot(vsub(q, aim), fwd), vdot(vsub(q, aim), right)
        if abs(yq) > y_lim + 1e-9:
            errs.append("%s 横向 %.2fm 超出两侧余量 %.2fm (D_SIDE)" % (nm, abs(yq), y_lim))
        if xq < x_rear_lim - 1e-9:
            errs.append("%s 在起飞点后方 %.2fm, 超出后方空地 (D_REAR=%.2f)"
                        % (nm, x_origin - xq, pv.D_REAR))
        r0 = math.hypot(q[0] - origin[0], q[1] - origin[1])
        if r0 > pv.GATE_XY_MAX:
            errs.append("%s 离起飞点 %.2fm > 水平范围 %.2fm (档案 xy_radius)" % (nm, r0, pv.GATE_XY_MAX))
    # 摆位路径 (直线 p_now→S) 全程在环前方: 两端都 ≤ REPO_X_MAX 即可 (线段是凸的)
    x_now = vdot(vsub(p_now, aim), fwd)
    if x_now > pv.REPO_X_MAX:
        errs.append("飞机现在离环平面只有 %.2fm (要 ≥ %.2fm 才允许摆位)"
                    % (-x_now, -pv.REPO_X_MAX))
    # ---- 撞纸速度 (滞后模型)
    if tr is not None:
        sim = simulate_lag(tr, pv.T_LAG)
        v_nom = v_at_vehicle_s(sim, -pv.L_F)
        v_worst = v_at_vehicle_s(sim, -pv.L_F + pv.E_ALONG)
        pass_act = pass_actual(sim, tr, pv)
        if v_worst < pv.V_TEAR_MIN - 1e-3:          # 滞后模型渐近到 v, 留一点数值余量
            errs.append("最坏撞纸速度 %.2f < 撕纸下限 %.2f m/s" % (v_worst, pv.V_TEAR_MIN))
    if errs:
        return errs
    P.v, P.d_run, P.S, P.E, P.E_end, P.traj = v, d_run, S, E, E_end, tr
    P.v_hit_nom, P.v_hit_worst = v_nom, v_worst
    P.veh_stop = sim[-1][1]
    P.brake_dist = v * pv.T_LAG + v * v / (2.0 * pv.BRAKE_ACC)
    P.pass_actual = pass_act
    P.ok = True
    return []


def pass_actual(sim, tr, p):
    """滞后模型下, 轨迹开始刹车 (t2) 那一刻飞机的机头过纸多远 = 飞机实际匀速过纸的距离。"""
    for t, s, _v in sim:
        if t >= tr.t2:
            return s + p.L_F
    return float("nan")


def plan_dash(aim, c, n, p_now, origin, p, virtual_offset=0.0):
    """
    aim     瞄点 (红点; 没有红点就是环心)
    c, n    环心、法向 (都可以是 None; n 是 None 时按视线方向进入)
    p_now   飞机现在的位置 (决定视线方向; 也是摆位路径的起点)
    origin  起飞点 (水平范围 GATE_XY_MAX 和 D_REAR 都相对它)
    virtual_offset > 0: 把环平面往进近侧挪这么多米当"虚拟环", 并把真纸面当成墙 ——
                        真冲刺, 但停在纸前 (T3 虚拟环测试)。

    速度从 DASH_V 开始, 不可行就每次降 DASH_V_STEP, 降到 DASH_V_MIN 还不行 → ok=False。
    起跑点 S 是动态的 (start_options): 飞机已经在 [最小助跑, D_RUN_MAX] 里就地起跑。
    """
    P = Plan()
    P.aim_tool = aim
    d, P.dir_why = entry_dir(aim, n, p_now, p.ENTRY_ANG_MAX)
    if d is None:
        P.errors.append("算不出进入方向 (飞机正在瞄点正上/正下方?)")
        return P
    P.dir = d
    # ---- 撞击点: 让针尖 (不是机体中心) 打到 aim  (2026-10-19)
    #   针尖在机体中心 右 NEEDLE_RIGHT / 上 NEEDLE_UP; 冲刺时机头 = 进入方向, 所以机体右 = 靶面右。
    #   AIM_TRIM_* 是看纸上破洞的经验修正: 洞要往右挪 x → AIM_TRIM_RIGHT += x (上同)。
    right, up, _ = entry_axes(d)
    off_r = p.NEEDLE_RIGHT - p.AIM_TRIM_RIGHT
    off_u = p.NEEDLE_UP - p.AIM_TRIM_UP
    if abs(off_r) > 1e-9 or abs(off_u) > 1e-9:
        aim = vsub(aim, vadd(vmul(right, off_r), vmul(up, off_u)))
        P.notes.append("撞击点: 机体中心瞄在目标 右 %+.0f / 上 %+.0f cm (针尖 右 %+.0f 上 %+.0f, "
                       "经验修正 右 %+.0f 上 %+.0f)"
                       % (-off_r * 100 + 0.0, -off_u * 100 + 0.0, p.NEEDLE_RIGHT * 100, p.NEEDLE_UP * 100,
                          p.AIM_TRIM_RIGHT * 100, p.AIM_TRIM_UP * 100))
    # ---- 机身安全区: 没有法向时用 −进入方向 当靶面法向 (以前没有 n 就不限, 首次撞纸时瞄点离环心 33cm)
    if c is not None:
        n_eff = n if n is not None else vmul(d, -1.0)
        aim2, P.aim_clamped = clamp_aim(aim, c, n_eff, p)
        if P.aim_clamped:
            P.tool_miss = vnorm(vsub(aim2, aim))
            P.warnings.append("瞄点离环心太远, 机身会擦环 —— 机体中心已拉回到安全区, "
                              "针尖会偏离目标 %.0fcm" % (P.tool_miss * 100))
            aim = aim2
    pv = p
    if virtual_offset > 0.0:
        # 虚拟环: 沿进入方向往回挪; 真纸面 (原 aim 处) 成了"墙"
        aim = vsub(aim, vmul(d, virtual_offset))
        pv = p.copy(D_BACK=virtual_offset)
        P.notes.append("虚拟环: 平面前移 %.2fm, 真纸面当墙" % virtual_offset)
    P.aim = aim

    # 环后空间: 降到 DASH_V_MIN 都停不下 → 直接放弃 (不用再试起跑点)
    if dash_v_room(pv) is None:
        st = dash_stop(pv.DASH_V_MIN, pv)
        P.errors.append("环后空间不够: 过纸冲刺 %.2fm、刹车超调 %.2fm, 降到 %.2fm/s 也要 D_BACK ≥ %.2fm (现在 %.2f%s) "
                        "—— 加大环后空间, 或减小 pass_run / back_margin"
                        % (pv.PASS_RUN, st["over"], pv.DASH_V_MIN, st["d_back_need"], pv.D_BACK,
                           ", 虚拟环偏移当墙" if virtual_offset > 0.0 else ""))
        return P
    w = pass_short(pv)
    if w:
        P.warnings.append(w)

    dz = aim[2] - origin[2]
    if not (pv.GATE_Z_MIN_DZ <= dz <= pv.GATE_Z_MAX_DZ):
        P.errors.append("瞄点高度 dz=%.2fm 不在 [%.2f, %.2f] (相对起飞点)"
                        % (dz, pv.GATE_Z_MIN_DZ, pv.GATE_Z_MAX_DZ))
        return P

    fwd, right = _ring_frame(d, n)
    # 起飞点在环系里的 x (负 = 在环前方); 后方空地从这里往后算
    x_origin = vdot(vsub(origin, aim), fwd)
    x_rear_lim = x_origin - pv.D_REAR + pv.M_WALL + pv.L_R
    y_lim = pv.D_SIDE - pv.W_HALF - pv.M_WALL

    # 飞机现在离 (虚拟) 瞄点多远 —— 沿进入方向量。动态起跑点用它 (见 start_options)
    dist_now = -vdot(vsub(p_now, aim), d)
    P.v_set = pv.DASH_V
    v = pv.DASH_V
    first_err, last_err = None, []
    while v >= pv.DASH_V_MIN - 1e-9:
        st = dash_stop(v, pv)
        P.s_stop, P.s_end, P.s_hi, P.tol, P.pass_lag = st["s_stop"], st["s_end"], st["s_hi"], st["margin"], st["lag"]
        P.s_lo = st["s_stop"]
        if st["margin"] < -1e-9:
            errs = ["环后空间: %.2fm/s 实际约停在 s=%+.2f (含超调 %.2f), 机头离墙只剩 %.2fm < %.2fm "
                    "(要 D_BACK ≥ %.2f)"
                    % (v, st["s_end"], st["over"], st["margin"] + pv.M_BACK, pv.M_BACK, st["d_back_need"])]
        else:
            errs = []
            for d_run, how in start_options(dist_now, v, pv):
                errs = _try_start(P, aim, d, fwd, right, x_origin, x_rear_lim, y_lim,
                                  p_now, origin, v, d_run, pv)
                if not errs:
                    P.d_run_min, P.start_how = d_run_for(v, pv), how
                    break
        if not errs:
            P.v_room = v
            if v < pv.DASH_V - 1e-9:
                P.slow_why = "; ".join(first_err or [])
                if int(pv.GATE_SLOW_POLICY) <= 0:
                    # ★不许降速: 规划失败, 告诉操作员卡在哪、最高能冲多快
                    P.ok = False
                    P.errors = ["按设定 %.2f m/s 不可行 (%s) —— 场地最高能冲 %.2f m/s。"
                                "要么放宽卡住的那项, 要么档案里 dash_v 改成 %.2f, "
                                "或者 allow_slow = 确认 / 是"
                                % (pv.DASH_V, P.slow_why, v, v)]
                    return P
                P.slow = True
                P.need_confirm = int(pv.GATE_SLOW_POLICY) == 1
                P.warnings.append("★ 降速: 冲刺速度从 %.2f 降到 %.2f m/s (按 %.2f 卡在: %s)%s"
                                  % (pv.DASH_V, v, pv.DASH_V, P.slow_why,
                                     " —— 冲刺前要按 Y 确认" if P.need_confirm else ""))
            P.speed_chain = [["DASH_V", pv.DASH_V]] + (
                [["场地/助跑限速", round(v, 2)]] if v < pv.DASH_V - 1e-9 else [])
            return P
        if first_err is None:
            first_err = errs
        last_err = errs
        v = round(v - pv.DASH_V_STEP, 6)
    P.ok = False
    P.slow_why = "; ".join(first_err or [])
    P.errors = ["按 %.2f m/s 不可行 (%s); 降到 %.2f m/s 仍不可行 (%s)"
                % (pv.DASH_V, P.slow_why, pv.DASH_V_MIN, "; ".join(last_err))]
    return P


def preflight_check(aim, c, n, origin, p, takeoff_h, virtual_offset=0.0, p_now=None):
    """
    起飞前 (2026-10-21): 用地面上看到的环位置先规划一遍, 假设从起飞点正上方 takeoff_h 处起跑。
    → dict
      ok_full   按 DASH_V 可行
      v_room    场地最高能冲多快 (None = 降到 DASH_V_MIN 也不行)
      why       按 DASH_V 卡在哪 (ok_full 时为空)
      xy_need   按 DASH_V 冲, xy_radius 至少要多少 (含刹车超调); None = 环后空间等别的项先卡住
      d_back_need  按 DASH_V 冲, 环后空间至少要多少
      summary   按 v_room 的计划摘要
    地面识别是从 4cm 高度、远处看的, 环位置可能差 10~20cm: 结果只用来提前发现"明显放不下", 起飞后照样重算。
    p_now 给了 = 从这个位置起跑 (手动段里按飞机当前位置实时检查用)。
    """
    if p_now is None:
        p_now = (origin[0], origin[1], origin[2] + takeoff_h)
    P = plan_dash(aim, c, n, p_now, origin, p.copy(GATE_SLOW_POLICY=2), virtual_offset=virtual_offset)
    out = {"ok_full": bool(P.ok and not P.slow), "v_room": P.v if P.ok else None,
           "why": P.slow_why or "; ".join(P.errors), "summary": P.summary(), "xy_need": None,
           "d_back_need": dash_stop(p.DASH_V, p.copy(D_BACK=virtual_offset) if virtual_offset > 0 else p)["d_back_need"]}
    # 不限水平范围再算一次 DASH_V: 制动终点离起飞点多远 = 需要的 xy_radius
    Q = plan_dash(aim, c, n, p_now, origin,
                  p.copy(GATE_SLOW_POLICY=0, GATE_XY_MAX=1e6, DASH_V_MIN=p.DASH_V), virtual_offset=virtual_offset)
    if Q.ok:
        out["xy_need"] = math.ceil(math.hypot(Q.E_end[0] - origin[0], Q.E_end[1] - origin[1]) * 10 + 1) / 10.0
    return out


RETREAT_ACC = 0.5       # m/s²，退回时的起步加速度 (小: 别把纸拽走 / 别把针别在纸里)
RETREAT_DEC = 0.8       # m/s²，退回时的刹车


def poke_end_s(p):
    """
    戳点终点 (机体中心的 s)。L_F = 中心 → 针尖; 针尖碰到纸面时机体中心在 s = −L_F。
      停在纸前 (老戳点): 针尖在纸面前方 POKE_GAP (负 = 扎进去 |POKE_GAP|)
      戳破 / 顶开 (poke_pierce): 针尖接触点再往里 POKE_DEPTH
    """
    if poke_pierce(p):
        return -p.L_F + p.POKE_DEPTH
    return -p.L_F - p.POKE_GAP


def plan_poke(aim, c, n, p_now, origin, p):
    """
    戳点: 慢速直线飞到"针尖离纸 POKE_GAP"处, 停留, 再沿原直线退回起跑点 S。
    和 plan_dash 共用: 进入方向 (entry_dir) / 撞击点偏移 (NEEDLE_*, AIM_TRIM_*) / 起跑点 (start_options) /
    场地检查。不同: ① 终点 = 纸前 POKE_GAP, 没有"刹停窗口 / 撕纸速度 / D_BACK"那套;
    ② 不穿环, 所以不做 clamp_aim (机身不用过环), 红点离环心多远都按红点瞄; ③ 多一条退回轨迹 traj_back。
    速度只用 POKE_V (不自动降速): 场地放不下就直接报不可行。
    """
    P = Plan()
    P.mode = "poke"
    P.aim_tool = aim
    d, P.dir_why = entry_dir(aim, n, p_now, p.ENTRY_ANG_MAX)
    if d is None:
        P.errors.append("算不出进入方向 (飞机正在瞄点正上/正下方?)")
        return P
    P.dir = d
    right, up, _ = entry_axes(d)
    off_r = p.NEEDLE_RIGHT - p.AIM_TRIM_RIGHT
    off_u = p.NEEDLE_UP - p.AIM_TRIM_UP
    if abs(off_r) > 1e-9 or abs(off_u) > 1e-9:
        aim = vsub(aim, vadd(vmul(right, off_r), vmul(up, off_u)))
        P.notes.append("撞击点: 机体中心瞄在目标 右 %+.0f / 上 %+.0f cm (针尖 右 %+.0f 上 %+.0f, "
                       "经验修正 右 %+.0f 上 %+.0f)"
                       % (-off_r * 100 + 0.0, -off_u * 100 + 0.0, p.NEEDLE_RIGHT * 100, p.NEEDLE_UP * 100,
                          p.AIM_TRIM_RIGHT * 100, p.AIM_TRIM_UP * 100))
    P.aim = aim
    v = p.POKE_V
    P.s_stop = poke_end_s(p)
    P.tip_gap = p.POKE_GAP
    P.s_lo = P.s_hi = P.s_stop
    P.tol = 0.0
    if poke_pierce(p):
        P.warnings.append("戳破/顶开: 以 %.2fm/s 压入纸面 %.0fcm 后停 (针露出长度要大于这个深度 + 纸位置误差, 否则机头 / 护桨会顶纸)"
                          % (poke_hit_v(p, p.POKE_V), p.POKE_DEPTH * 100))
        if p.POKE_DEPTH > 0.15:
            P.warnings.append("压入深度 %.0fcm 较大: 纸被顶开后可能贴到护桨 / 螺旋桨上" % (p.POKE_DEPTH * 100))
    elif p.POKE_GAP < -0.10:
        P.warnings.append("扎入深度 %.0fcm: 位置设定点比纸面深这么多, 针被纸挡住时飞机会持续往前顶"
                          % (-p.POKE_GAP * 100))

    dz = aim[2] - origin[2]
    if not (p.GATE_Z_MIN_DZ <= dz <= p.GATE_Z_MAX_DZ):
        P.errors.append("瞄点高度 dz=%.2fm 不在 [%.2f, %.2f] (相对起飞点)"
                        % (dz, p.GATE_Z_MIN_DZ, p.GATE_Z_MAX_DZ))
        return P

    fwd, rgt = _ring_frame(d, n)
    x_origin = vdot(vsub(origin, aim), fwd)
    x_rear_lim = x_origin - p.D_REAR + p.M_WALL + p.L_R
    y_lim = p.D_SIDE - p.W_HALF - p.M_WALL
    dist_now = -vdot(vsub(p_now, aim), d)
    errs = []
    for d_run, how in start_options(dist_now, v, p):
        errs = []
        S = vadd(aim, vmul(d, -d_run))
        E = vadd(aim, vmul(d, P.s_stop))
        try:
            tr = make_poke_traj(d_run, v, p)
            back = Mirror(Traj1D(-P.s_stop, d_run, p.POKE_RETREAT_V, RETREAT_ACC, RETREAT_DEC))
        except ValueError as e:
            errs.append(str(e))
            continue_ = True
        else:
            continue_ = False
        for nm, q in (("起跑点 S", S), ("终点 E", E)):
            xq, yq = vdot(vsub(q, aim), fwd), vdot(vsub(q, aim), rgt)
            if abs(yq) > y_lim + 1e-9:
                errs.append("%s 横向 %.2fm 超出两侧余量 %.2fm (D_SIDE)" % (nm, abs(yq), y_lim))
            if xq < x_rear_lim - 1e-9:
                errs.append("%s 在起飞点后方 %.2fm, 超出后方空地 (D_REAR=%.2f)"
                            % (nm, x_origin - xq, p.D_REAR))
            r0 = math.hypot(q[0] - origin[0], q[1] - origin[1])
            if r0 > p.GATE_XY_MAX:
                errs.append("%s 离起飞点 %.2fm > 水平范围 %.2fm" % (nm, r0, p.GATE_XY_MAX))
        x_now = vdot(vsub(p_now, aim), fwd)
        if x_now > p.REPO_X_MAX:
            errs.append("飞机现在离环平面只有 %.2fm (要 ≥ %.2fm 才允许摆位)" % (-x_now, -p.REPO_X_MAX))
        if errs or continue_:
            continue
        sim = simulate_lag(tr, p.T_LAG)
        P.v, P.d_run, P.S, P.E, P.traj, P.traj_back = v, d_run, S, E, tr, back
        P.d_run_min, P.start_how = d_run_for(v, p), how
        P.v_hit_nom = v_at_vehicle_s(sim, -p.L_F)
        P.v_hit_worst = v_at_vehicle_s(sim, -p.L_F + p.E_ALONG)
        P.veh_stop = sim[-1][1]
        P.brake_dist = v * p.T_LAG + v * v / (2.0 * p.BRAKE_ACC)
        P.speed_chain = [["POKE_V", p.POKE_V]]
        P.pierce = poke_pierce(p)
        if P.pierce:
            P.speed_chain.append(["POKE_HIT_V", poke_hit_v(p, v)])
            P.hit_v = poke_hit_v(p, v)
            P.v_hit_nom = P.v_hit_worst = P.hit_v        # 爬行段匀速: 接触速度 = hit_v (±滞后 ≈ 0)
        elif p.POKE_GAP < 0 and P.v_hit_nom < 0.3:
            P.warnings.append("扎入时接触速度只有 %.2f m/s —— 针可能戳不透纸 (改用戳破模式: POKE_GAP=0 且 POKE_HIT_V>0)"
                              % P.v_hit_nom)
        P.ok = True
        return P
    P.errors = ["戳点不可行: " + "; ".join(errs)]
    return P


def static_preview(p):
    """
    不知道环在哪时 (dry-run / 配置页) 的派生量: 只用速度和场地参数,
    假设正对进入。→ dict, 字段对应 INTERFACE §4 的 derived.gate。
    """
    if p.GATE_POKE:
        v = p.POKE_V
        d_run = d_run_for(v, p)
        s_stop = poke_end_s(p)
        pierce = poke_pierce(p)
        out = {"mode": "poke", "dash_v": v, "poke_v": v, "poke_gap": p.POKE_GAP,
               "poke_retreat_v": p.POKE_RETREAT_V, "pierce": pierce,
               "poke_hit_v": poke_hit_v(p, v) if pierce else 0.0,
               "poke_creep": p.POKE_CREEP if pierce else 0.0,
               "poke_depth": p.POKE_DEPTH if pierce else 0.0,
               "dash_acc": p.DASH_ACC, "brake_acc": p.BRAKE_ACC,
               "brake_dist": v * p.T_LAG + v * v / (2.0 * p.BRAKE_ACC),
               "d_align": d_run, "d_commit": d_run, "d_run_max": max(d_run, p.D_RUN_MAX),
               "overshoot": s_stop, "stop_window": [s_stop, s_stop], "tol": 0.0,
               "r_safe": r_safe(p)}
        try:
            tr = make_poke_traj(d_run, v, p)
            back = Traj1D(-s_stop, d_run, p.POKE_RETREAT_V, RETREAT_ACC, RETREAT_DEC)
            sim = simulate_lag(tr, p.T_LAG)
            out.update(s_brake=tr.s_b, t_dash=tr.T, t_back=back.T,
                       v_hit=poke_hit_v(p, v) if pierce else v_at_vehicle_s(sim, -p.L_F),
                       v_hit_worst=poke_hit_v(p, v) if pierce else v_at_vehicle_s(sim, -p.L_F + p.E_ALONG),
                       veh_stop=sim[-1][1])
        except ValueError as e:
            out["error"] = str(e)
        return out
    # 冲刺: 按场地允许的最高速度预览 (DASH_V 放不下时就是规划会降到的那一档)
    v_room = dash_v_room(p)
    v = v_room if v_room is not None else p.DASH_V
    st = dash_stop(v, p)
    d_run = d_run_for(v, p)
    out = {"mode": "dash", "dash_v": p.DASH_V, "v_room": v_room, "v_plan": v,
           "slow_policy": int(p.GATE_SLOW_POLICY), "brake_over": st["over"], "s_end": st["s_end"],
           "brake_over_t": p.BRAKE_OVER_T,
           "dash_acc": p.DASH_ACC, "brake_acc": p.BRAKE_ACC,
           "brake_dist": v * p.T_LAG + v * v / (2.0 * p.BRAKE_ACC),
           "pass_run": p.PASS_RUN, "pass_lag": st["lag"], "back_margin": p.M_BACK,
           "d_back_need": dash_stop(p.DASH_V, p)["d_back_need"],     # 要按 DASH_V 冲需要多大环后空间
           "d_align": d_run, "d_commit": d_run, "d_run_max": max(d_run, p.D_RUN_MAX),
           "overshoot": st["s_stop"], "stop_window": [st["s_end"], st["s_hi"]], "tol": st["margin"],
           "r_safe": r_safe(p)}
    try:
        tr = Traj1D(-d_run, st["s_stop"], v, p.DASH_ACC, p.BRAKE_ACC)
        sim = simulate_lag(tr, p.T_LAG)
        out.update(s_brake=tr.s_b, t_dash=tr.T,
                   v_hit=v_at_vehicle_s(sim, -p.L_F),
                   v_hit_worst=v_at_vehicle_s(sim, -p.L_F + p.E_ALONG),
                   pass_actual=pass_actual(sim, tr, p),
                   veh_stop=sim[-1][1])
    except ValueError as e:
        out["error"] = str(e)
    return out


# =============================================================================
# 估计: ALIGN 窗口取样 → 锁定值
# =============================================================================
def median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return float("nan")
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def pct_range(xs, lo=0.1, hi=0.9):
    """p90 − p10 (比 max−min 抗单个野值)。"""
    s = sorted(xs)
    if len(s) < 2:
        return 0.0
    i0 = int(round(lo * (len(s) - 1)))
    i1 = int(round(hi * (len(s) - 1)))
    return s[i1] - s[i0]


def fuse_samples(samples, p_ref, d=None):
    """
    samples: [{"c": (3), "aim": (3)|None, "n": (3)|None, "sig": (h, v, along)|None, ...}]
    → dict(c, aim, n, spread_cm, n_spread_deg, k)  逐分量中位数; 法向平均后归一化,
      符号翻到指向 p_ref (飞机) 一侧。
    d (进入方向) 给了 → 另给靶面拆分: spread_hva / aim_spread_hva (cm), sig_hva (各轴 σ 中位数, cm)
    """
    k = len(samples)
    out = {"k": k}
    if k == 0:
        return out
    c = tuple(median([s["c"][i] for s in samples]) for i in range(3))
    out["c"] = c
    out["spread_cm"] = max(pct_range([s["c"][i] for s in samples]) for i in range(3)) * 100.0
    aims = [s["aim"] for s in samples if s.get("aim") is not None]
    if aims:
        out["aim"] = tuple(median([a[i] for a in aims]) for i in range(3))
        out["aim_spread_cm"] = max(pct_range([a[i] for a in aims]) for i in range(3)) * 100.0
    ns = [vunit(s["n"]) for s in samples if s.get("n") is not None]
    if ns:
        ref = ns[0]
        acc = (0.0, 0.0, 0.0)
        for v in ns:
            acc = vadd(acc, v if vdot(v, ref) >= 0 else vmul(v, -1.0))
        nn = vunit(acc)
        if vdot(vsub(p_ref, c), nn) < 0:
            nn = vmul(nn, -1.0)
        out["n"] = nn
        out["n_spread_deg"] = max(ang_deg(v if vdot(v, nn) >= 0 else vmul(v, -1.0), nn)
                                  for v in ns)
    if d is not None:
        out["spread_hva"] = spread_hva([s["c"] for s in samples], d)
        if aims:
            out["aim_spread_hva"] = spread_hva(aims, d)
        sg = [s["sig"] for s in samples if s.get("sig")]
        if sg:
            out["sig_hva"] = tuple(median([x[i] for x in sg]) for i in range(3))
    return out


# =============================================================================
# 离线例子
# =============================================================================
DEFAULTS = dict(
    DASH_V=2.5, DASH_V_MIN=1.2, DASH_V_STEP=0.3, DASH_ACC=2.0, BRAKE_ACC=2.5, T_LAG=0.2,
    D_STRAIGHT=1.5, D_ALIGN_MIN=2.8, ENTRY_ANG_MAX=20.0, E_ALONG=0.25, V_TEAR_MIN=1.2,
    M_PASS=0.2, M_WALL=0.3, M_BACK=0.3, PASS_RUN=1.0, BRAKE_OVER_T=0.3, GATE_SLOW_POLICY=1, L_F=0.28, L_R=0.28, W_HALF=0.28, R_IN=0.6, AIM_MARGIN=0.10,
    D_BACK=2.0, D_REAR=2.0, D_SIDE=2.0, GATE_XY_MAX=8.0, REPO_X_MAX=-1.5,
    GATE_Z_MIN_DZ=0.5, GATE_Z_MAX_DZ=3.0, D_RUN_MAX=4.0,
    H_HALF=0.11, NEEDLE_RIGHT=0.0, NEEDLE_UP=0.0, AIM_TRIM_RIGHT=0.0, AIM_TRIM_UP=0.0,
    GATE_POKE=False, POKE_V=1.0, POKE_GAP=0.5, POKE_RETREAT_V=0.4,
    POKE_HIT_V=0.5, POKE_CREEP=0.25, POKE_DEPTH=0.10,
)

if __name__ == "__main__":
    p = Params(**DEFAULTS)
    print("静态预览 (D_BACK=%.1f):" % p.D_BACK)
    for k, v in sorted(static_preview(p).items()):
        print("   %-12s %s" % (k, ("%.3f" % v) if isinstance(v, float) else v))
    print()
    origin = (0.0, 0.0, 0.0)
    c = (5.0, 0.3, 1.5)
    n = vunit((-1.0, -0.2, 0.0))
    aim = vadd(c, (0.0, 0.08, -0.05))
    print("各速度需要的环后空间 (PASS_RUN=%.2f, M_BACK=%.2f, T_LAG=%.2f, BRAKE_ACC=%.1f):"
          % (p.PASS_RUN, p.M_BACK, p.T_LAG, p.BRAKE_ACC))
    for v in (2.5, 2.2, 2.0, 1.6, 1.2):
        st = dash_stop(v, p)
        print("   v=%.1f  s_b=%+.2f  s_stop=%+.2f  含超调 %+.2f  D_BACK ≥ %.2f"
              % (v, st["s_b"], st["s_stop"], st["s_end"], st["d_back_need"]))
    print()
    for db, pol in ((2.5, 0), (2.5, 1), (4.0, 0)):
        for vmax in (1.5, 2.0, 2.5):
            P = plan_dash(aim, c, n, (0.0, 0.0, 1.5), origin, p.copy(DASH_V=vmax, D_BACK=db, GATE_SLOW_POLICY=pol))
            print("降速策略=%d" % pol, end=" ")
            print("D_BACK=%.1f DASH_V=%.1f  %s" % (db, vmax, P.summary()))
            for w in P.warnings:
                print("     ⚠ " + w)
    P = plan_dash(aim, c, n, (0.0, 0.0, 1.5), origin, p, virtual_offset=2.0)
    print("虚拟环 2m   %s" % P.summary())
    for w in P.notes + P.warnings:
        print("     · " + w)
    print()
    pp = p.copy(GATE_POKE=True)
    for gap in (0.5, 0.2, 0.0, -0.05):
        P = plan_poke(aim, c, n, (0.0, 0.0, 1.5), origin, pp.copy(POKE_GAP=gap))
        print("戳点 gap=%+.2f  %s" % (gap, P.summary()))
        for w in P.notes + P.warnings:
            print("     · " + w)
    print("戳点静态预览:", {k: (round(v, 3) if isinstance(v, float) else v)
                       for k, v in static_preview(pp).items()})
