# -*- coding: utf-8 -*-
"""demo_track_sim.py —— 红点追踪全流程模拟（贴近实际的版本 v2）

═══════════════════════════════════════════════════════════════════
相对 v1 改了什么（都是被现实纠正的）
───────────────────────────────────────────────────────────────────
1. **删掉"尺寸停滞"判据**：前提错了。锁错目标后往前飞，那个像素照样
   会变大。所以"d_px 在涨"只证明"我在接近它"，不能证明"锁对了"。
2. **加上真正的目标身份维持（data association）**：丢失后不是
   "重新检测选最大的"，而是"在上次位置附近，按位置+尺寸打分选最像的"。
3. **加歧义判定**：最佳与次佳得分差 <20% → 提示人，但不停机。
4. **检测走真实链路**：每帧先 JPEG 压缩，再 HSV 阈值 + 连通域。
5. **加遮挡模拟（O 键）**：目标被挡 1.5 秒，验证重捕获选得对不对。

═══════════════════════════════════════════════════════════════════
按键
───────────────────────────────────────────────────────────────────
  鼠标单击   选中候选（仅 ACQUIRE / PAUSE）
  回车       确认目标
  W A S D    手动接管
  T          恢复自动追踪
  空格       软暂停 / 继续
  O          ★模拟遮挡 1.5 秒
  X / Esc    退出

用法：
  python scripts/demo_track_sim.py                       交互
  python scripts/demo_track_sim.py --auto                自动确认
  python scripts/demo_track_sim.py --auto --occlude-at 8 第 8 秒自动遮挡
"""
import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402
from mcpv.tracker_core import TemplateTracker              # noqa: E402

# ═══════════════════════════════════════════════════════════════════
#  可调参数（全部集中在这里，改这一块就够）
# ═══════════════════════════════════════════════════════════════════

# ── ① 相机与目标 ───────────────────────────────────────────────────
F_PX_1920 = 1566.1      # 焦距（像素）@1920 宽。换分辨率时自动等比换算
WIN_W, WIN_H = 960, 540  # 演示窗口尺寸（= 处理分辨率）

# ★ 红圆实际直径（米）。这是**唯一需要按实物改的参数**！
#     A4 纸最大能印 20 cm → 用 0.20
#     A3 纸可印 26 cm     → 改成 0.26
#   改这个值后，下面所有像素阈值会自动跟着换算，不用手动改。
MARKER_REAL_M = 0.20

# ── ② 交接与安全距离（用"米"定义，自动换算成像素）──────────────────
HANDOFF_DIST_M   = 6.5   # 到这个距离就停住、交接给破窗模块
                         #   6.5 m 比 10 m 更近，更利于破窗；
                         #   冲过头（5~6 m）也没关系，不回退
HARD_FLOOR_DIST_M = 3.0  # ★ 硬底线：小于这个距离强制刹车+轻退
                         #   这一条优先级最高，唯一目的是别撞墙

# ── ③ 速度与刹车 ───────────────────────────────────────────────────
V_MAX_MS   = 4.0        # 接近段最大速度（m/s）
V_MIN_MS   = 1.5        # 末端最小速度（m/s），太慢演示不够流畅
A_DEC_MS2  = 2.0        # 减速度（m/s²）—— 用于刹车预测
                        #   真机上应按实测校准，或用帧间 d_px 变化率在线估计

# ── ④ 跟踪器（模板 NCC + 背景光流）───────────────────────────────
TRK_MIN_SCORE = 0.35    # 模板匹配分低于此 → 判"丢了"
                        #   偏低更不容易中断（人能接管兜底）
TRK_MIN_RED   = 0.15    # 匹配中心红色占比下限 → 防止匹配到背景
TRK_CTX_SCALE = 1.6     # 模板包含的上下文倍数（>1 才能靠背景区分同类目标）
TRK_PAD_BASE  = 40      # 搜索窗基础外扩（像素）
TRK_PAD_GROW  = 20.0    # 每丢失 1 秒额外扩大（像素）

# ── ⑤ 丢失后的处理 ────────────────────────────────────────────────
LOSE_SLOW1_S = 1.0      # 丢失超过 1 s → 减速到 50%
LOSE_SLOW2_S = 3.0      # 丢失超过 3 s → 减速到 20%
LOSE_PAUSE_S = 20.0     # 丢失超过 20 s → 悬停等人（放很宽，人能接管）

# ── ⑥ VLM 语义指认（起飞悬停时调一次，之后不再调用）──────────────
USE_VLM_HINT = True     # 是否启用 VLM 指认 + 候选排序
VLM_NOISE_PX = 55       # 模拟 VLM 粗框的定位噪声（实测误差 86~119 px）
VLM_DESC = "窗户上的红色圆点"   # VLM 会返回的描述（真机是模型原文）

# ── ⑦ 演示场景（只影响模拟，不影响真机）───────────────────────────
SIM_START_M   = 50.0    # 模拟起飞距离
SIM_END_M     = 6.5     # 模拟最终距离
SIM_APPROACH_S = 13.0   # 模拟接近耗时
JPEG_Q = 85             # 模拟 vstream 的 JPEG 压缩质量

# ═══════════════════════════════════════════════════════════════════
#  派生量（不要手改，改上面的参数即可）
# ═══════════════════════════════════════════════════════════════════
F = F_PX_1920 * WIN_W / 1920.0                    # 当前分辨率下的焦距
K = WIN_W / 1920.0                                # 分辨率缩放比

def dpix_at(dist_m):
    """距离(米) → 红圆在画面里的像素直径。"""
    return F * MARKER_REAL_M / max(dist_m, 0.1)

D_HANDOFF_PX = dpix_at(HANDOFF_DIST_M)            # 交接阈值（像素）
D_FLOOR_PX   = dpix_at(HARD_FLOOR_DIST_M)         # 硬底线（像素）
D_START, D_END = SIM_START_M, SIM_END_M
APPROACH_S = SIM_APPROACH_S

# 关联/搜索（v2 检测+关联用；模板跟踪启用后仅作兜底）
SEARCH_R = [(0.3, 20), (1.0, 80), (3.0, 150), (5.0, 300)]
W_POS, W_SIZE = 0.6, 0.4
SCORE_MIN = 0.50
AMBIG_RATIO = 0.20


def text(img, s, xy, scale=0.6, color=(235, 235, 235), thick=1):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick,
                cv2.LINE_AA)


def draw_dot_subpixel(frame, cx, cy, d_px, color=(30, 30, 205), S=4):
    """精确直径画圆（4x 超采样），避免整数量化。"""
    r = max(0.6, d_px / 2.0)
    half = int(r) + 3
    size = 2 * half + 1
    x0, y0 = int(cx) - half, int(cy) - half
    H, W = frame.shape[:2]
    if x0 < 0 or y0 < 0 or x0 + size > W or y0 + size > H:
        return
    big = np.zeros((size * S, size * S, 3), np.uint8)
    cv2.circle(big, (int(round((cx - x0) * S)), int(round((cy - y0) * S))),
               int(round(r * S)), color, -1, lineType=cv2.LINE_AA)
    small = cv2.resize(big, (size, size), interpolation=cv2.INTER_AREA)
    m = small.any(axis=2)
    reg = frame[y0:y0 + size, x0:x0 + size]
    reg[m] = small[m]
    frame[y0:y0 + size, x0:x0 + size] = reg


def detect_reds(bgr, min_area=3):
    """真机同款链路：JPEG 压缩 → HSV 阈值 → 连通域。"""
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_Q])
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        a = int(st[i][4])
        if a < min_area:
            continue
        x, y, w, h = st[i][0], st[i][1], st[i][2], st[i][3]
        out.append({"cx": float(cen[i][0]), "cy": float(cen[i][1]), "area": a,
                    "d": 2 * math.sqrt(a / math.pi),
                    "box": (int(x), int(y), int(w), int(h))})
    out.sort(key=lambda o: -o["d"])
    return out


def draw_corners(img, box, color, pad=4):
    x, y, w, h = box
    x0, y0, x1, y1 = x - pad, y - pad, x + w + pad, y + h + pad
    L = max(6, int(min(w, h) * 0.35))
    for (ax, ay, dx, dy) in ((x0, y0, 1, 1), (x1, y0, -1, 1),
                             (x0, y1, 1, -1), (x1, y1, -1, -1)):
        cv2.line(img, (ax, ay), (ax + dx * L, ay), color, 2)
        cv2.line(img, (ax, ay), (ax, ay + dy * L), color, 2)


def search_radius(lost_s):
    for t, r in SEARCH_R:
        if lost_s <= t:
            return r
    return SEARCH_R[-1][1]


def score_cand(c, pu, pv, d_old, radius):
    dist = math.hypot(c["cx"] - pu, c["cy"] - pv)
    s_pos = max(0.0, 1.0 - dist / max(radius, 1e-6))
    if d_old and d_old > 0 and c["d"] > 0:
        s_size = max(0.0, 1.0 - abs(math.log(c["d"] / d_old)) / math.log(1.5))
    else:
        s_size = 0.5
    return W_POS * s_pos + W_SIZE * s_size


def rank_by_vlm_hint(cands, hint_cx, hint_cy):
    """按 VLM 粗框的距离给候选排序（只排序，不定位）。

    VLM 的框误差约 86~119 px，所以只看"离它多近"，
    不能拿它的坐标当结果。
    """
    if not cands:
        return []
    dmax = max(1e-6, max(math.hypot(c["cx"] - hint_cx, c["cy"] - hint_cy)
                        for c in cands))
    scored = []
    for c in cands:
        d = math.hypot(c["cx"] - hint_cx, c["cy"] - hint_cy)
        s_pos = 1.0 - d / dmax                      # 越近越高
        # 尺寸项权重很小：红点 8px、红砖 ~40px，靠尺寸分不出来，主要靠位置
        s_size = min(1.0, c["d"] / 30.0)
        scored.append((0.85 * s_pos + 0.15 * s_size, c))
    scored.sort(key=lambda x: -x[0])
    return scored


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--pick", type=int, default=-1)
    ap.add_argument("--record", default="")
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--occlude-at", type=float, default=-1)
    ap.add_argument("--noise-dots", type=int, default=2)
    ap.add_argument("--close-dot", type=float, default=0,
                    help="在目标旁边 N 像素处再放一个同样大小的红点")
    ap.add_argument("--brick", action="store_true",
                    help="墙上画一块大红砖（测试候选排序是否必要）")
    ap.add_argument("--no-vlm", action="store_true", help="关闭 VLM 指认")
    ap.add_argument("--marker-cm", type=float, default=0,
                    help="红圆实际直径(cm)，覆盖 MARKER_REAL_M；20=A4, 26=A3")
    ap.add_argument("--debug", action="store_true")
    a = ap.parse_args()

    global MARKER_REAL_M, D_HANDOFF_PX, D_FLOOR_PX
    if a.marker_cm > 0:
        MARKER_REAL_M = a.marker_cm / 100.0
        D_HANDOFF_PX = dpix_at(HANDOFF_DIST_M)
        D_FLOOR_PX = dpix_at(HARD_FLOOR_DIST_M)

    base = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    if base is None:
        raise SystemExit("读不到背景图")
    base = cv2.resize(base, (WIN_W, WIN_H), interpolation=cv2.INTER_AREA)
    base = (base.astype(np.float32) * 0.80).clip(0, 255).astype(np.uint8)

    print(f"窗口 {WIN_W}x{WIN_H}  等效焦距 {F:.0f}  "
          f"红圆 {MARKER_REAL_M*100:.0f} cm")
    print(f"  交接点 {HANDOFF_DIST_M:.1f} m → d_px {D_HANDOFF_PX:.1f}   "
          f"硬底线 {HARD_FLOOR_DIST_M:.1f} m → d_px {D_FLOOR_PX:.1f}")

    noise = [(int(WIN_W * 0.15), int(WIN_H * 0.80)),
             (int(WIN_W * 0.86), int(WIN_H * 0.18)),
             (int(WIN_W * 0.30), int(WIN_H * 0.10))][:a.noise_dots]

    state = "ACQUIRE"
    t0 = t_prev = time.time()
    t_enter = time.time()
    prog = [0.0]
    selected = None
    confirmed = False
    manual = pause = False
    occl_until = 0.0
    warn = ""
    trans = []
    speed = 1.0
    track = {"u": None, "v": None, "d": 0.0, "hist": [],
             "vel": (0.0, 0.0), "last_seen": 0.0, "lost": 0.0,
             "pu": None, "pv": None, "score": 0.0, "ambig": 0.0}
    cands = []
    last_cands = []
    tx_now = [0]
    ty_now = [0]
    sel_ref = [None]       # 保存选中的候选本身（不是下标）
    vlm_hint = [None]      # (cx, cy) 模拟的 VLM 粗框中心
    vlm_done = [False]
    ranked = []            # 排序后的候选
    trk = TemplateTracker(ctx_scale=TRK_CTX_SCALE,
                      base_pad=TRK_PAD_BASE,
                      pad_grow=TRK_PAD_GROW)
    trk.MIN_SCORE = TRK_MIN_SCORE
    trk.MIN_RED_RATIO = TRK_MIN_RED
    frame_now = [None]           # 当前帧（do_confirm 要用）

    writer = None
    if a.record:
        writer = cv2.VideoWriter(a.record, cv2.VideoWriter_fourcc(*"mp4v"),
                                 25, (WIN_W, WIN_H))

    def enter(ns, why=""):
        nonlocal state, t_enter
        if ns != state:
            trans.append({"t": round(time.time() - t0, 2), "from": state,
                          "to": ns, "why": why})
            print(f"  [{time.time()-t0:5.1f}s] {state:9s} -> {ns:9s}  {why}")
            state = ns
            t_enter = time.time()

    def do_confirm():
        nonlocal selected
        c0 = sel_ref[0]
        if c0 is None:
            return False
        track.update({"u": c0["cx"], "v": c0["cy"], "d": c0["d"],
                      "hist": [(c0["cx"], c0["cy"], c0["d"])],
                      "vel": (0.0, 0.0), "last_seen": time.time(),
                      "lost": 0.0, "pu": c0["cx"], "pv": c0["cy"],
                      "score": 1.0, "ambig": 0.0})
        # ★ 确认时在**当前帧**重新对齐：选中到确认隔了 ~1 秒，
        #   目标已经移动 8~10 px，直接用手选时的坐标会让模板中心偏离红点，
        #   之后红色校验每次都采不到红（实测 nred=0）而误判丢失。
        cx_, cy_ = c0["cx"], c0["cy"]
        if cands:
            near = min(cands, key=lambda q: math.hypot(q["cx"] - cx_,
                                                       q["cy"] - cy_))
            if math.hypot(near["cx"] - cx_, near["cy"] - cy_) < 30:
                cx_, cy_ = near["cx"], near["cy"]
                c0 = near
        trk.init(frame_now[0], (cx_, cy_),
                 half_size=max(8, int(c0["d"] * 1.1)), d_px=c0["d"])
        dx_ = c0["cx"] - tx_now[0]
        dy_ = c0["cy"] - ty_now[0]
        print(f"  确认 d={c0['d']:.1f}px 位置=({c0['cx']:.0f},{c0['cy']:.0f}) "
              f"真值=({tx_now[0]},{ty_now[0]}) 偏差={math.hypot(dx_, dy_):.0f}px")
        return True

    def on_mouse(e, x, y, flags, param):
        nonlocal selected
        if e == cv2.EVENT_LBUTTONDOWN and state in ("ACQUIRE", "PAUSE"):
            for i, c in enumerate(last_cands):
                bx, by, bw, bh = c["box"]
                if bx - 8 <= x <= bx + bw + 8 and by - 8 <= y <= by + bh + 8:
                    sel_ref[0] = c
                    selected = i
                    print(f"  选中 d={c['d']:.1f}px "
                          f"位置=({c['cx']:.0f},{c['cy']:.0f})")

    print("模拟开始（X 退出，O 模拟遮挡）")
    cv2.namedWindow("track-sim", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("track-sim", on_mouse)

    while True:
        now = time.time()
        if a.duration and (now - t0) > a.duration:
            print("  达到 --duration，正常退出")
            break
        dt = max(1e-3, now - t_prev)
        t_prev = now
        el = now - t0

        if a.occlude_at > 0 and abs(el - a.occlude_at) < dt and occl_until < now:
            occl_until = now + 1.5
            print(f"  [自动遮挡] {el:.1f}s 起 1.5s")

        # ── 造图 ──────────────────────────────────────
        if state in ("APPROACH", "RECAPTURE") and not manual and not pause:
            prog[0] = min(1.0, prog[0] + dt * speed / APPROACH_S)
        dist = D_START - (D_START - D_END) * prog[0]
        d_px = F * MARKER_REAL_M / dist

        frame = base.copy()
        tx = int(WIN_W * 0.62 + 16 * math.sin(el * 1.1))
        ty = int(WIN_H * 0.40 + 9 * math.cos(el * 1.7))
        tx_now[0], ty_now[0] = tx, ty
        occluded = now < occl_until
        if not occluded:
            draw_dot_subpixel(frame, tx, ty, d_px)
        else:
            cv2.rectangle(frame, (tx - 16, ty - 16), (tx + 16, ty + 16),
                          (58, 62, 58), -1)
        for (nx, ny) in noise:
            draw_dot_subpixel(frame, nx, ny, 3.2, (40, 45, 175))
        if a.brick:
            # 墙上一块红色砖/招牌：比红点大得多，纯按尺寸排序会选错
            cv2.rectangle(frame, (int(WIN_W * 0.20), int(WIN_H * 0.52)),
                          (int(WIN_W * 0.20) + 46, int(WIN_H * 0.52) + 22),
                          (40, 45, 165), -1)
        if a.close_dot > 0 and not occluded:
            draw_dot_subpixel(frame, tx + a.close_dot, ty, d_px)
        if occluded:
            text(frame, "OCCLUDED", (12, WIN_H - 70), 0.8, (60, 200, 255), 2)

        frame_now[0] = frame
        cands = detect_reds(frame)
        last_cands = cands

        # ── 状态机 ────────────────────────────────────
        if state == "ACQUIRE":
            # VLM 指认：悬停稳定后调一次（这里用"真值+噪声"模拟）
            if (USE_VLM_HINT and not a.no_vlm and not vlm_done[0]
                    and el > 1.2 and cands):
                nz = np.random.normal(0, VLM_NOISE_PX, 2)
                vlm_hint[0] = (tx + float(nz[0]), ty + float(nz[1]))
                vlm_done[0] = True
                print(f"  [VLM 一次调用] 描述=\"{VLM_DESC}\"  "
                      f"粗框中心=({vlm_hint[0][0]:.0f},{vlm_hint[0][1]:.0f})"
                      f"  真值=({tx},{ty})")

            if vlm_hint[0] is not None:
                ranked = rank_by_vlm_hint(cands, *vlm_hint[0])
            else:
                ranked = [(0.0, c) for c in cands]

            if (a.auto or a.pick >= 0) and el > 1.5 and ranked and selected is None:
                # 有 VLM 提示时选排序第 1；否则按尺寸选最大
                if a.pick >= 0 and a.pick < len(ranked):
                    selected = a.pick
                else:
                    selected = 0
                # selected 是"排序后"的下标，换算回 cands 下标
                if vlm_hint[0] is not None and ranked:
                    sel_ref[0] = ranked[selected][1]
                elif selected < len(cands):
                    sel_ref[0] = cands[selected]
                else:
                    sel_ref[0] = cands[0]
                selected = 0
            if (a.auto or a.pick >= 0) and el > 2.4 and sel_ref[0] is not None \
                    and not confirmed:
                if do_confirm():
                    confirmed = True
                    enter("TRACK", "自动确认")

        elif state == "TRACK":
            if now - t_enter > 0.8:
                enter("APPROACH", "跟踪稳定")

        elif state in ("APPROACH", "RECAPTURE"):
            # ★ 用模板+光流跟踪器保持身份（不再用"检测+关联"）
            if frame_now[0] is not None and trk.tmpl is not None:
                ok, cen, score = trk.update(frame_now[0], dt, d_est=track['d'])
                if ok:
                    track["u"], track["v"] = cen
                    track["score"] = score
                    track["lost"] = 0.0
                    # 尺寸仍用检测量（跟踪器只给位置）
                    near = min(cands, key=lambda c: math.hypot(
                        c["cx"] - cen[0], c["cy"] - cen[1])) if cands else None
                    if near and math.hypot(near["cx"] - cen[0],
                                           near["cy"] - cen[1]) < max(20, near["d"]):
                        track["hist"].append((near["cx"], near["cy"], near["d"]))
                        track["hist"] = track["hist"][-5:]
                        h = track["hist"]
                        track["d"] = sum(q[2] for q in h) / len(h)
                    speed = 1.0
                    warn = ""
                    if state == "RECAPTURE":
                        enter("APPROACH", f"重捕获成功 score={score:.2f}")
                else:
                    if a.debug:
                        d_ = getattr(trk, "dbg", {})
                        print(f"    [dbg] score={d_.get('score',0):.2f} "
                              f"red_ok={d_.get('red_ok')} "
                              f"nred={d_.get('nred',0):.0f} "
                              f"expect={d_.get('expect',0):.1f} "
                              f"cen={d_.get('cen')}")
                    cen = trk.predict(frame_now[0], dt)
                    track["u"], track["v"] = cen
                    track["lost"] = trk.lost_s
                    if track["lost"] > 0.3 and state == "APPROACH":
                        enter("RECAPTURE", f"丢失 {track['lost']:.1f}s，光流预测中")
                    speed = (1.0 if track["lost"] < 1.0 else
                             0.5 if track["lost"] < 3.0 else 0.2)
                    warn = (f"LOST {track['lost']:.1f}s  score={score:.2f}"
                            f"  R={trk._pad()}px -- still flying")
                    if track["lost"] > 20.0:
                        enter("PAUSE", f"丢失 {track['lost']:.1f}s 仍找不到")

            if state == "APPROACH" and track["d"] >= D_HANDOFF_PX:
                enter("HANDOFF", f"d_px={track['d']:.1f} >= {D_HANDOFF_PX:.1f}")

        # ── 绘制 ──────────────────────────────────────
        vis = frame.copy()
        # 候选按"排序后"的顺序编号显示
        show = [c for _, c in ranked][:6] if ranked else cands[:6]
        for k, c in enumerate(show):
            is_sel = (selected is not None and k < len(cands)
                      and cands[selected] is c)
            col = (80, 240, 90) if is_sel else (
                (60, 220, 255) if k == 0 else (150, 170, 190))
            draw_corners(vis, c["box"], col)
            tag = f"{k+1}" + ("*" if k == 0 and vlm_hint[0] else "")
            text(vis, tag, (c["box"][0] - 2, c["box"][1] - 8), 0.55, col, 2)

        # VLM 粗框（虚线感：画四角，颜色偏紫）
        if vlm_hint[0] is not None:
            hx, hy = int(vlm_hint[0][0]), int(vlm_hint[0][1])
            cv2.circle(vis, (hx, hy), 70, (200, 130, 60), 1)
            text(vis, f"VLM: {VLM_DESC}", (12, WIN_H - 96), 0.62,
                 (200, 180, 120), 2)

        if state == "RECAPTURE" and track["pu"] is not None:
            cv2.circle(vis, (int(track["pu"]), int(track["pv"])),
                       int(search_radius(track["lost"])), (60, 200, 255), 1)

        cv2.rectangle(vis, (0, 0), (WIN_W, 44), (28, 28, 28), -1)
        scol = {"ACQUIRE": (60, 220, 255), "TRACK": (80, 240, 90),
                "APPROACH": (80, 240, 90), "RECAPTURE": (60, 200, 255),
                "HANDOFF": (0, 140, 255), "PAUSE": (60, 60, 240),
                "MANUAL": (60, 60, 240)}.get(state, (235, 235, 235))
        text(vis, f"STATE: {state}", (12, 31), 0.9, scol, 2)

        rows = [
            f"d_px   {track['d']:5.1f}  (handoff {D_HANDOFF_PX:.1f})",
            f"dist   {(F*MARKER_REAL_M/track['d'] if track['d']>0 else 0):5.1f} m",
            f"speed  {(4.0*speed if state in ('APPROACH','RECAPTURE') and not manual and not pause else 0):.1f} m/s",
            f"lost   {track['lost']:4.1f} s   R={search_radius(track['lost'])} px",
            f"score  {track['score']:.2f}   lost {track['lost']:.1f}s",
            f"cand   {len(cands)}   " +
            ("VLM 已指认" if vlm_hint[0] else "VLM 未用"),
        ]
        cv2.rectangle(vis, (0, 50), (370, 50 + 24 * len(rows) + 10),
                      (28, 28, 28), -1)
        for i, s in enumerate(rows):
            text(vis, s, (12, 72 + i * 24), 0.58, (235, 235, 235), 1)

        if warn:
            cv2.rectangle(vis, (0, WIN_H - 66), (WIN_W, WIN_H - 32),
                          (0, 0, 120), -1)
            text(vis, warn, (10, WIN_H - 44), 0.58, (60, 220, 255), 2)

        cv2.rectangle(vis, (0, WIN_H - 28), (WIN_W, WIN_H), (28, 28, 28), -1)
        text(vis, "click=pick  ENTER=confirm  WASD=manual  T=resume  "
                  "SPACE=pause  O=occlude  X=quit",
             (10, WIN_H - 8), 0.5, (170, 170, 170), 1)

        cv2.imshow("track-sim", vis)
        if writer:
            writer.write(vis)

        kb = cv2.waitKey(28) & 0xFF
        if kb in (ord("x"), 27):
            break
        elif kb in (13, 10) and state in ("ACQUIRE", "PAUSE") and sel_ref[0] is not None:
            if do_confirm():
                confirmed = True
                enter("TRACK", "人工确认")
        elif kb in (ord("w"), ord("a"), ord("s"), ord("d")):
            if state in ("APPROACH", "RECAPTURE", "TRACK"):
                manual = True
                enter("MANUAL", "人接管")
        elif kb == ord("t") and state == "MANUAL":
            manual = False
            enter("APPROACH", "T 恢复")
        elif kb == 32:
            pause = not pause
            print(f"  {'暂停' if pause else '继续'}")
        elif kb == ord("o"):
            occl_until = now + 1.5
            print(f"  [手动遮挡] {el:.1f}s 起 1.5s")

    if writer:
        writer.release()
    cv2.destroyAllWindows()

    print("\n状态转移：")
    for r in trans:
        print(f"  {r['t']:6.2f}s  {r['from']:9s} -> {r['to']:9s}  {r['why']}")
    out = ROOT / "eval" / "demo_out" / f"track_sim_{datetime.now():%H%M%S}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"transitions": trans}, ensure_ascii=False,
                              indent=2), encoding="utf-8")
    print(f"日志 {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
