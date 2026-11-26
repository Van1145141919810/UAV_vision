# -*- coding: utf-8 -*-
"""bench_physics_stop.py —— 有惯性时能不能"停在合适的像素大小"

═══════════════════════════════════════════════════════════════════
用户提的两个实际问题
───────────────────────────────────────────────────────────────────
1. **飞机有惯性，能在像素合适时就停吗？**
2. **冲过头、像素变更大了，怎么处理？**

═══════════════════════════════════════════════════════════════════
物理模型（关键：指令发出后还会滑行）
───────────────────────────────────────────────────────────────────
    v_{k+1} = v_k + clip(v_target - v_k, -a_dec·dt, +a_acc·dt)
    D_{k+1} = D_k - v_{k+1}·dt
    d_px = f · REAL / D

固定减速度 a 下，从 v 刹到 0 需要 s = v²/(2a)。
30 kg 机在 4 m/s、a=2 m/s² 时 s = 4 m —— 相对 10 m 交接点**巨大**。

所以：
  · P0 等 d_px 到阈值才发停 → 之后还会滑 v²/2a，冲过头
  · P1 按刹停预测提前减速 → 停下时正好在阈值
  · P2 提前减速 + 超调回退 → 如果还是过了，轻微后退

═══════════════════════════════════════════════════════════════════
"""
import math

F1920, W_IMG = 1566.1, 960
F = F1920 * W_IMG / 1920.0
REAL = 0.20
D_HANDOFF_1920 = 30.0
D_AIM_M = 6.5        # 实际瞄准的交接距离（用户：更近更好）
D_HANDOFF = D_HANDOFF_1920 * W_IMG / 1920.0
D_AIM_PX = F * REAL / D_AIM_M          # 6.5 m 对应的像素直径
DT = 1.0 / 10.4
V_MAX = 4.0
A_DEC = 2.0
A_ACC = 1.5
V_MIN = 1.5


def dpix(D):
    return F * REAL / max(D, 0.5)


def run(policy="P1", v_max=V_MAX, a_dec=A_DEC, verbose=False,
        backoff=True, a_actual=None):
    D, v, t = 50.0, 0.0, 0.0
    peak = 0.0
    stopped_at = None
    backed = 0.0
    hist = []
    while t < 60.0 and D > 1.0:
        d = dpix(D)
        peak = max(peak, d)

        if stopped_at is not None:
            if v < 0.05:
                # 已经停住：若冲过头太多，考虑后退
                if backoff and d > D_AIM_PX * 2.0 and policy == "P2":
                    if backed < 4.0:
                        v_t = -V_MIN          # 轻微后退
                    else:
                        break
                else:
                    break
            else:
                v_t = 0.0
        else:
            s_stop = (v * v) / (2 * a_dec)
            D_pred = D - s_stop
            d_pred = dpix(D_pred) if D_pred > 1 else 1e9

            if policy == "P0":
                v_t = 0.0 if d >= D_AIM_PX else v_max
                if d >= D_AIM_PX:
                    stopped_at = (D, d)
            else:
                # 预测"现在全力减速会停在哪"：一旦停住时 d_px 会超阈值，就减速
                limit = D_AIM_PX * (1.02 if policy == "P2" else 1.0)
                if d_pred >= limit:
                    v_t = 0.0
                    stopped_at = (D, d_pred)
                else:
                    v_t = min(v_max, max(V_MIN,
                                         math.sqrt(max(0.0, 2 * a_dec *
                                                       max(0.02, D -
                                                           F * REAL / limit)))))

        # 积分
        dv = v_t - v
        a_eff = a_dec if a_actual is None else a_actual
        lim = (A_ACC if dv > 0 else a_eff) * DT
        v += max(-lim, min(lim, dv))
        D -= v * DT
        if v < 0:
            backed += -v * DT
        t += DT
        hist.append((t, D, v, d))
        if verbose and abs(t * 2 - round(t * 2)) < 1e-6:
            print(f"    t={t:5.1f} D={D:6.2f} v={v:5.2f} d_px={d:6.1f}")

    return {"D_end": D, "d_end": dpix(D), "peak": peak, "t": t,
            "backed": backed, "hist": hist}


if __name__ == "__main__":
    print(f"瞄准 d_px={D_AIM_PX:.1f}（{D_AIM_M:.1f} m）  "
          f"v_max={V_MAX}  减速 {A_DEC} m/s²  帧率 10.4")
    print(f"{'策略':<26}{'最终距离':<11}{'最终d_px':<11}{'冲过':<9}{'耗时'}")
    print("-" * 70)
    for tag, kw in (("P0 到点才停（无预测）", dict(policy="P0")),
                    ("P1 提前刹车", dict(policy="P1")),
                    ("P2 提前刹车 + 超调回退", dict(policy="P2"))):
        r = run(**kw)
        over = (r["d_end"] - D_AIM_PX) / D_AIM_PX * 100
        print(f"{tag:<26}{r['D_end']:<11.2f}{r['d_end']:<11.1f}"
              f"{over:+8.0f}%  {r['t']:.1f}s")

    print("\n--- 假设失配：规划按 2.0 m/s²，实际只有 1.2 m/s²（风/载荷）---")
    print(f"{'策略':<26}{'最终距离':<11}{'最终d_px':<11}{'冲过':<9}{'后退'}")
    print("-" * 74)
    for tag, kw in (("P1 提前刹车", dict(policy="P1")),
                    ("P2 提前刹车 + 超调回退", dict(policy="P2"))):
        kw2 = dict(kw); kw2["a_actual"] = 1.2
        r = run(**kw2)
        over = (r["d_end"] - D_AIM_PX) / D_AIM_PX * 100
        print(f"{tag:<26}{r['D_end']:<11.2f}{r['d_end']:<11.1f}"
              f"{over:+8.0f}%  {r['backed']:.2f} m")
