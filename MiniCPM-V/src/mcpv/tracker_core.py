# -*- coding: utf-8 -*-
"""tracker_core.py —— 轻量跟踪器 v2：模板 NCC（多尺度）+ 背景光流预测

═══════════════════════════════════════════════════════════════════
专注的问题（只有一个）
───────────────────────────────────────────────────────────────────
**目标丢失后，怎么认出"还是刚才那一个"。**

不是检测阈值调优，不是飞控，不是语义——只做身份保持。

═══════════════════════════════════════════════════════════════════
v2 相对 v1 修了三处（实测 33 px 偏差的原因）
───────────────────────────────────────────────────────────────────
1. **搜索窗随丢失时长放大**
   v1 固定 pad=40，预测偏了就永远找不回来。
   v2: pad = base + 20×丢失秒数（上限 120）。

2. **光流特征点定期刷新**
   v1 只在初始化时取一次，特征漂出画面后失锁。
   v2: 每 10 帧重取一次（摊销 ~1 ms/帧）。

3. **模板多尺度匹配**
   v1 模板固定在锁定时的大小，目标接近时会变大，
   尺度不匹配导致 NCC 峰值下降。
   v2: 在 [0.85, 1.0, 1.18, 1.4] 四档尺度上匹配，取最优。

═══════════════════════════════════════════════════════════════════
成本（960x540，实测）
───────────────────────────────────────────────────────────────────
    光流 30 点          ~1.6 ms
    多尺度模板匹配      ~6 ms（4 档）
    合计               ~8 ms/帧，占 10.4 fps 预算的 8%
"""
import cv2
import numpy as np


class TemplateTracker:
    """模板匹配 + 背景光流预测的轻量跟踪器（v2）。

    用法：
        tr = TemplateTracker()
        tr.init(frame, (cx, cy), half_size=24, d_px=8)
        ok, (cx, cy), score = tr.update(frame, dt)   # 每帧
        tr.predict(frame, dt)                        # 丢失时用
    """

    # ★ 实测：多尺度反而更差（13~106 px），因为别的尺度能匹配到
    #   附近的窗框结构。改回单尺度 + 定期刷新模板。
    SCALES = (1.0,)
    REFEATURE_EVERY = 10
    TEMPLATE_REFRESH = 15      # 高分时每 15 帧用当前帧重取模板
    ROLLING_MIN_SCORE = 0.55   # 匹配分高于此才滚动更新模板（防漂移）
    # ★ 用户明确：最终都能人工接管，所以阈值一律放宽，优先不中断
    MIN_SCORE = 0.35           # 判"还在"的匹配分下限（原 0.45）
    MIN_RED_RATIO = 0.15       # 中心红色占比下限（原 0.25）

    def __init__(self, ctx_scale=1.6, base_pad=40, pad_grow=20.0,
                 max_pad=120, decay=0.9):
        self.ctx_scale = ctx_scale
        self.base_pad = base_pad
        self.pad_grow = pad_grow
        self.max_pad = max_pad
        self.decay = decay
        # ★ 预测位置相对"最后一次成功匹配点"的最大漂移。
        #   不限的话，丢失时靠光流一帧帧外推，会跑到画面另一头：
        #   现场录像里十字飘到墙上、离真实红点 250 px（实测）。
        self.max_drift = 120.0
        self.last_good = None
        self.tmpl = None
        self.tmpl_off = (0, 0)
        self.centre = None
        self.half = None
        self.lost_s = 0.0
        self.flow = (0.0, 0.0)
        self.pts_prev = None
        self.gray_prev = None
        self._n = 0
        self._last_upd = None

    # ── 内部：光流 ──────────────────────────────────────
    def _flow_step(self, gray):
        if self.gray_prev is None or self.pts_prev is None:
            self.gray_prev = gray
            return (0.0, 0.0)
        p1, st, _ = cv2.calcOpticalFlowPyrLK(
            self.gray_prev, gray, self.pts_prev, None,
            winSize=(15, 15), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        if p1 is None or st is None:
            self.gray_prev = gray
            return (0.0, 0.0)
        good = st.ravel() == 1
        if good.sum() < 5:
            self.gray_prev = gray
            return (0.0, 0.0)
        d = (p1[good] - self.pts_prev[good]).reshape(-1, 2)
        dx = float(np.median(d[:, 0]))
        dy = float(np.median(d[:, 1]))
        self.pts_prev = p1[good].reshape(-1, 1, 2)
        self.gray_prev = gray
        return (dx, dy)

    def _refresh_features(self, gray):
        H, W = gray.shape
        mask = np.full_like(gray, 255)
        if self.centre:
            cx, cy = int(self.centre[0]), int(self.centre[1])
            r = int((self.half or 20) * 2.2)
            mask[max(0, cy - r):min(H, cy + r),
                 max(0, cx - r):min(W, cx + r)] = 0
        pts = cv2.goodFeaturesToTrack(gray, 30, 0.01, 10, mask=mask)
        self.pts_prev = pts
        self.gray_prev = gray

    # ── 内部：模板 ──────────────────────────────────────
    def _grab_template(self, frame):
        cx, cy = int(round(self.centre[0])), int(round(self.centre[1]))
        hh = int(round((self.half or 20) * self.ctx_scale))
        x0, y0 = max(0, cx - hh), max(0, cy - hh)
        x1 = min(frame.shape[1], cx + hh)
        y1 = min(frame.shape[0], cy + hh)
        if x1 - x0 < 10 or y1 - y0 < 10:
            self.tmpl = None
            return
        self.tmpl = frame[y0:y1, x0:x1].copy()
        self.tmpl_off = (cx - x0, cy - y0)

    def _pad(self):
        return int(min(self.max_pad, self.base_pad + self.pad_grow * self.lost_s))

    # ── 对外 ────────────────────────────────────────────
    def init(self, frame, centre, half_size=24, d_px=None):
        self.centre = (float(centre[0]), float(centre[1]))
        self.half = int(half_size)
        self.lost_s = 0.0
        self._n = 0
        self._last_upd = None
        self.flow = (0.0, 0.0)
        self.last_good = self.centre
        self._grab_template(frame)
        self._refresh_features(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

    def predict(self, frame, dt=0.1):
        """丢失时：按相机运动外推位置（带衰减）。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dx, dy = self._flow_step(gray)
        self.lost_s += dt
        k = self.decay ** max(0.0, self.lost_s)
        self.flow = (-dx * k, -dy * k)
        if self.centre:
            nx = self.centre[0] + self.flow[0]
            ny = self.centre[1] + self.flow[1]
            if self.last_good is not None:
                dx, dy = nx - self.last_good[0], ny - self.last_good[1]
                dd = (dx * dx + dy * dy) ** 0.5
                if dd > self.max_drift:      # ★ 钳在实测点附近
                    k = self.max_drift / dd
                    nx = self.last_good[0] + dx * k
                    ny = self.last_good[1] + dy * k
            self.centre = (nx, ny)
        self._n += 1
        if self._n % self.REFEATURE_EVERY == 0:
            self._refresh_features(gray)
        return self.centre

    def update(self, frame, dt=0.1, d_est=None):
        """正常更新：多尺度模板匹配。返回 (ok, centre, score)。"""
        if self.tmpl is None or self.centre is None:
            return False, self.centre, 0.0
        self._n += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 先用光流把搜索中心挪到"相机运动后的位置"
        dx, dy = self._flow_step(gray)
        px = self.centre[0] - dx
        py = self.centre[1] - dy

        pad = self._pad()
        best = (-1.0, None, None)      # (score, centre, scale)
        for sc in self.SCALES:
            th, tw = self.tmpl.shape[:2]
            tw2, th2 = max(8, int(tw * sc)), max(8, int(th * sc))
            if tw2 > frame.shape[1] or th2 > frame.shape[0]:
                continue
            t2 = cv2.resize(self.tmpl, (tw2, th2), interpolation=cv2.INTER_AREA)
            ox, oy = self.tmpl_off[0] * sc, self.tmpl_off[1] * sc
            x0 = int(round(px - ox)) - pad
            y0 = int(round(py - oy)) - pad
            x1, y1 = x0 + tw2 + 2 * pad, y0 + th2 + 2 * pad
            H, W = frame.shape[:2]
            x0c, y0c = max(0, x0), max(0, y0)
            x1c, y1c = min(W, x1), min(H, y1)
            if x1c - x0c < tw2 or y1c - y0c < th2:
                continue
            reg = frame[y0c:y1c, x0c:x1c]
            res = cv2.matchTemplate(reg, t2, cv2.TM_CCOEFF_NORMED)
            _, maxv, _, maxloc = cv2.minMaxLoc(res)
            if maxv > best[0]:
                best = (maxv, (x0c + maxloc[0] + ox, y0c + maxloc[1] + oy), sc)

        score, cen, sc = best

        # ★ 颜色校验：模板含大量上下文，遮挡后可能整块匹配到背景
        #   （实测 score 会到 1.00 却偏离 60 px）。所以要求匹配中心是红的。
        red_ok = False
        if cen is not None:
            cx, cy = int(cen[0]), int(cen[1])
            # ★ 取样窗口必须贴合目标实际大小：
            #   50 m 时红点只有 ~12 像素，若按模板尺寸取样，
            #   红色占比永远达不到阈值 → 会被全部拒绝（实测立刻判丢失）。
            dd = d_est if (d_est and d_est > 2) else (self.half * 1.6)
            # 取样窗口至少 5px 半径：4px 的小点配 7x7 窗口时，
            # 匹配稍偏 2~3 px 就采不到红色，会误判为丢失（实测踩过）。
            h2 = max(5, int(round(dd * 1.0)))
            x0c, y0c = max(0, cx - h2), max(0, cy - h2)
            x1c = min(frame.shape[1], cx + h2)
            y1c = min(frame.shape[0], cy + h2)
            patch = frame[y0c:y1c, x0c:x1c]
            if patch.size:
                ph = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
                red = cv2.bitwise_or(
                    cv2.inRange(ph, (0, 90, 45), (14, 255, 255)),
                    cv2.inRange(ph, (166, 90, 45), (180, 255, 255)))
                nred = red.sum() / 255.0
                # 期望面积 ≈ π(d/2)^2；只要达到期望的 25% 就算"中心是红的"
                expect = max(4.0, 3.1416 * (dd / 2.0) ** 2)
                red_ok = nred >= max(2.0, self.MIN_RED_RATIO * expect)

        ok = score >= self.MIN_SCORE and red_ok
        self.dbg = {'score': score, 'red_ok': red_ok,
                    'nred': float(nred), 'expect': float(expect),
                    'cen': cen}
        if ok:
            self.centre = (float(cen[0]), float(cen[1]))
            self.last_good = self.centre      # ★ 记录最后一次"实测"位置
            self.lost_s = 0.0
            if self._n % self.REFEATURE_EVERY == 0:
                self._refresh_features(gray)
            # ★ 目标随接近变大 → 模板必须跟着更新，否则匹配分一路掉到 0.3，
            #   被误判成"丢失"。实测：固定在锁定时取的模板，8 秒后就失配。
            #   做法：每次匹配够好（>=0.55）就用当前帧重取模板 —— 滚动更新。
            #   只在好匹配上更新，避免把错误位置学进模板。
            if score >= self.ROLLING_MIN_SCORE:
                self._grab_template(frame)
        else:
            self.lost_s += dt
        self._last_upd = (score, sc)
        return ok, self.centre, float(score)

    def reset(self):
        self.tmpl = None
        self.centre = None
        self.pts_prev = None
        self.gray_prev = None
        self.lost_s = 0.0
