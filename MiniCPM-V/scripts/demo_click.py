# -*- coding: utf-8 -*-
"""点击 → 飞行 的交互演示（只算指令，不连飞机）。

## 状态机（三层，防误点）

    没选中  ──单击──►  预选(黄)  ──双击──►  锁定(红)  ──回车──►  已发送
                         │                  │
                         └──再单击别处──────┘（自动解锁，只保留一个目标）
                        Esc / r 任何时候回"没选中"

    · 单击：只算方向/步长，**不调模型、不发任何东西**（零后果）
    · 双击：红点才会跑 VLM 判 A/B；判 A 才切"精确距离 + 小步长"
    · 回车：写 record_<时间戳>.json + 追加 session.jsonl（演示模式，不接飞控）

## 数据流

    图片(data/samples) ─► CV: 窗框 + 红点候选 ─► 你点 ─► aim 算位移
                                                      │
                                          VLM 语义(仅双击红点/发送时)
                                                      ▼
                                        data/results/record_*.json

## 每张图只算一次检测
    窗框/红点候选按文件名缓存；来回切图命中缓存是 0.01s 级。
    （早先每帧重算，4096×3072 的图要 1 秒，按 n/p 会假死。）

## 性能常识
    · 窗框在 2000px 宽的缩略图上算，再放大回原图坐标
    · 红点候选的环形带统计只对圆度最高的前 MAX_STATS 个做
"""

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mcpv import config as C                            # noqa: E402
from mcpv.aim import Aimer                              # noqa: E402
from mcpv.cv import find_candidates, imread_unicode     # noqa: E402
from mcpv.windows import find_windows, estimate_range   # noqa: E402
from mcpv.scene import describe_target, describe_marker  # noqa: E402
from mcpv.report import make_record, save_record        # noqa: E402

MAX_W, MAX_H = 1400, 900
HIT_PAD = 8          # 命中容差（显示坐标，像素）


def fit(img):
    h, w = img.shape[:2]
    s = min(MAX_W / w, MAX_H / h, 1.0)
    return s


def hit_test(x, y, dot_boxes, win_boxes, pad=HIT_PAD):
    """返回 ('dot'|'win', box) 或 None。红点框优先。"""
    for kind, boxes in (("dot", dot_boxes), ("win", win_boxes)):
        for b in boxes:
            bx, by, bw, bh = b
            if bx - pad <= x <= bx + bw + pad and by - pad <= y <= by + bh + pad:
                return kind, b
    return None


def wheel_delta(flags):
    """从 OpenCV 的 flags 里取滚轮增量（向上为正，一格约 +120）。

    为什么不用 cv2.getMouseWheelDelta：
        有的 OpenCV 版本没有这个函数（用户实测 AttributeError）。
        老办法是取 flags 的高 16 位；Python 里可能是负数，
        按 16 位有符号数还原一下。
    """
    fn = getattr(cv2, "getMouseWheelDelta", None)
    if fn is not None:
        try:
            return fn(flags)
        except Exception:
            pass
    raw = (int(flags) >> 16) & 0xFFFF
    return raw - 0x10000 if raw >= 0x8000 else raw


class Demo:
    def __init__(self, files):
        self.files = files
        self.idx = 0
        self.show_win = True
        self.img = None
        self.s = 1.0
        self.aim = None
        self.win_boxes = []
        self.dot_boxes = []
        self.sel = None        # {"kind","rect","point","cmd","armed"}
        self.cache = {}        # 文件名 -> (win_boxes, dot_cands)，避免来回切图重复算
        self.dirty = True      # True 才重绘：每帧都 resize 会挤掉鼠标事件
        self.zoom = 1.0        # 1.0 = 整图铺满；滚轮放大看小框
        self.center = None     # 视口中心（原图坐标）
        self.view_size = None  # 视口像素尺寸
        self.pan_anchor = None # 右键拖动时的起点（视口坐标）
        self.last_click = None # 屏幕上显示最后一次点击，便于确认事件有没有到
        self.flash = ""

    # ---------- 载入 ----------
    def load(self):
        """载入当前图：算窗框、红点候选、整帧估距；命中缓存则秒回。"""
        p = self.files[self.idx % len(self.files)]
        self.img = imread_unicode(p)
        h, w = self.img.shape[:2]
        self.s = fit(self.img)
        self.view_size = (int(w * self.s), int(h * self.s))
        self.zoom = 1.0                 # 换图复位，免得在上一张的放大位置迷路
        self.center = (w / 2.0, h / 2.0)
        self.aim = Aimer.from_config(w, h)

        key = p.name
        if key in self.cache:                       # 缓存命中：不重算
            self.win_boxes, self.dot_cands = self.cache[key]
            self.facade_box = self._union_box(self.win_boxes)
            self.win_range, _n, _s = estimate_range(
                self.win_boxes, self.aim.fx, w, C.WINDOW_REAL_W_M)
            self.dot_boxes = [self._tight(c) for c in self.dot_cands]
            print(f"[{key}] 缓存命中：窗框 {len(self.win_boxes)}，"
                  f"红点候选 {len(self.dot_cands)}")
            self.sel = None
            return

        t0 = cv2.getTickCount()
        k = min(1.0, 2000.0 / w)                 # 缩小再算，省时间
        small = cv2.resize(self.img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA) if k < 1.0 else self.img
        self.win_boxes = [(int(x / k), int(y / k), int(bw / k), int(bh / k))
                          for (x, y, bw, bh) in find_windows(small)]
        # 界面用"贴身的框"，不要用给 VLM 的那个大裁剪框（否则会吞掉窗框）
        # 展示 5 个（不是 3 个）：目标在不同距离下像素差很大，
        # 3 个名额会把真目标挤掉（test3 的红点 110px、磁钉 24px，排名会互相挤）。
        # 排序仍按 cdiff（不加人为惩罚，免得为演示好看而骗自己）。
        self.dot_cands = find_candidates(self.img)[:C.DISPLAY_CANDIDATES]
        self.dot_boxes = [self._tight(c) for c in self.dot_cands]
        self.cache[key] = (self.win_boxes, self.dot_cands)   # 存缓存
        self.facade_box = self._union_box(self.win_boxes)    # 立面外接框
        self.win_range, n_used, spread = estimate_range(
            self.win_boxes, self.aim.fx, w, C.WINDOW_REAL_W_M)

        # 先打文件名，再打细节——否则日志里"估距"那行会跑到上一张图下面
        ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        print(f"\n[{p.name}] {w}x{h}  窗框 {len(self.win_boxes)} 个，"
              f"红点候选 {len(self.dot_boxes)} 个（计算 {ms:.0f} ms）")
        if self.win_range:
            print(f"      整帧估距 {self.win_range:.1f} m"
                  f"（用了 {n_used} 个框，离散度 {spread:.2f}）")
        else:
            print(f"      整帧估距不可信（{n_used} 个合格框），只用固定步长")
        self.sel = None
        self.dirty = True

    @staticmethod
    def _tight(c):
        side = min(160, max(24, int(c.diameter_px * 1.6)))
        return (int(c.cx - side / 2), int(c.cy - side / 2), side, side)

    # ---------- 选点 ----------
    def select(self, sx, sy):
        """sx,sy 是显示坐标。"""
        u, v = self.view_to_image(sx, sy)   # 显示坐标 → 原图坐标（含缩放/平移）
        _, _, kx, _, _, _ = self._tf()
        hit = hit_test(u, v, self.dot_boxes, self.win_boxes, pad=HIT_PAD / max(kx, 1e-6))
        if hit:
            kind, (bx, by, bw, bh) = hit
            cu, cvv = bx + bw / 2.0, by + bh / 2.0
        else:
            kind, cu, cvv = "point", u, v
        rng = None
        if hit:
            if kind == 'dot':
                for c in self.dot_cands:
                    if abs(c.cx - cu) < 2 and abs(c.cy - cvv) < 2:
                        if c.diameter_px > 3:
                            rng = self.aim.range_from_size(
                                c.diameter_px, C.MARKER_REAL_M, self.aim.fx)
                        break
            else:
                # 窗户：不听单个框的，用整帧所有窗框的中位数（抗"窗+墙连成一块"）
                rng = getattr(self, "win_range", None)
        else:
            # 点空白：只有在"立面区域"里才敢用整帧估距；
            # 点到天空/地面就报未知（单目没有那部分的深度）。
            rng = getattr(self, "win_range", None) if self._in_facade(u, v) else None
        if rng is not None and not (1.0 < rng < C.MAX_RANGE_M):
            rng = None
        cmd = self.aim.command(cu, cvv, step_m=C.CLICK_STEP_M,
                               distance_m=rng, standoff_m=C.CLICK_STANDOFF_M)
        self.sel = {"kind": kind, "point": (cu, cvv), "cmd": cmd, "armed": False}
        self.last_click = (int(sx), int(sy))
        print(f"  预选 {kind} @({cu:.0f},{cvv:.0f})  {cmd.describe()}")

    def _crop(self, point, half=200):
        cu, cvv = point
        h, w = self.img.shape[:2]
        x0, y0 = max(0, int(cu - half)), max(0, int(cvv - half))
        x1, y1 = min(w, int(cu + half)), min(h, int(cvv + half))
        p = self.img[y0:y1, x0:x1]
        if p.size == 0:
            p = self.img
        return cv2.resize(p, (384, 384), interpolation=cv2.INTER_LANCZOS4)

    def arm(self):
        """双击 = 锁定。红点会先问 VLM 是不是标记，判 A 才进精确模式。

        为什么双击才调模型：单击只做预选（零后果），不该花 1~2 秒。
        """
        if not self.sel:
            return
        self.sel["armed"] = True

        # 红点：锁定时先让 VLM 判一次"是不是标记"
        # 只有判 A 才用 0.20 m 算精确距离、给小步长；判 B 退回保守大步长
        if self.sel["kind"] == "dot":
            try:
                v = describe_marker(self._crop(self.sel["point"]))
            except Exception as e:
                v = {"marker": "?", "raw": f"{type(e).__name__}: {e}"}
            self.sel["marker"] = v.get("marker")
            # 找"被选中的那个候选"，不是固定取第一个
            cu, cvv = self.sel["point"]
            cand = min(self.dot_cands,
                       key=lambda c: (c.cx - cu) ** 2 + (c.cy - cvv) ** 2)
            on_win_ok = (v.get("on_window") == "是") or (not C.REQUIRE_ON_WINDOW)
            if v.get("marker") == "A" and on_win_ok and cand.diameter_px > 3:
                rng = self.aim.range_from_size(
                    cand.diameter_px, C.MARKER_REAL_M, self.aim.fx)
                if 0.8 < rng < C.MAX_RANGE_M:
                    self.sel["cmd"] = self.aim.command(
                        self.sel["point"][0], self.sel["point"][1],
                        step_m=C.CLICK_STEP_M, distance_m=rng,
                        standoff_m=C.CLICK_STANDOFF_M)
                    tag = "贴窗" if v.get("on_window") == "是" else "贴纸/非窗"
                    self.flash = (f"已锁定·确认是标记（{tag}）  距离约 {rng:.1f} m，"
                                  f"按 [回车] 发送")
                else:
                    self.flash = f"标记已确认但估距异常({rng:.1f}m)，用保守步长"
            else:
                self.flash = ("⚠ 不像目标（标记=B 或 未贴窗），用保守大步长；"
                              "可用 Esc 取消")
            print(f"  ★ 锁定 dot  VLM={v.get('marker')}  → {self.sel['cmd'].describe()}")
            return

        self.flash = "已锁定：按 [回车] 发送，[Esc] 取消"
        print(f"  ★ 锁定 {self.sel['kind']}  → {self.sel['cmd'].describe()}")

    def fire(self):
        """回车 = 发送。写 record JSON 并追加 session.jsonl。

        演示模式下**只存档、不发飞控**；接真机时这里是发 step_cmd 的地方。
        """
        if not self.sel or not self.sel["armed"]:
            self.flash = "先双击锁定，再按回车"
            return
        cu, cvv = self.sel["point"]
        half = 200
        x0, y0 = max(0, int(cu - half)), max(0, int(cvv - half))
        x1 = min(self.img.shape[1], int(cu + half))
        y1 = min(self.img.shape[0], int(cvv + half))
        patch = self.img[y0:y1, x0:x1]
        patch = cv2.resize(patch if patch.size else self.img, (384, 384),
                           interpolation=cv2.INTER_LANCZOS4)
        try:
            if self.sel["kind"] == "dot":
                scene = describe_marker(patch)      # 红点：判是不是标记
            else:
                scene = describe_target(patch)      # 窗户/任意点：判是不是窗
        except Exception as e:
            scene = {"error": f"{type(e).__name__}: {e}"}
        sel = {k: v for k, v in self.sel.items() if k not in ("cmd", "armed")}
        h, w = self.img.shape[:2]
        rec = make_record(str(self.files[self.idx % len(self.files)]),
                          (w, h), sel, self.sel["cmd"], scene)
        path = save_record(rec)
        if self.sel["kind"] == "dot":
            verdict = {"A": "是标记", "B": "不是标记", "?": "没看懂"}.get(
                scene.get("marker"), scene.get("marker"))
            self.flash = f"已发送  红点判定={verdict}  → {path.name}"
        else:
            self.flash = (f"已发送  窗={scene.get('window_state')} "
                          f"反光={scene.get('glare')}  → {path.name}")
        print(f"  ▶ 发送: {self.sel['cmd'].to_dict()}")
        print(f"  ▶ 语义: {scene}")
        print(f"  ▶ 已存: {path}")

    # ---------- 视口变换（缩放 / 平移）----------
    def _tf(self):
        """返回 (x0, y0, kx, ky, cw, ch)：把原图的一块矩形映射到视口。

        为什么需要这层：滚轮放大后画面只显示原图的一小块，
        画框和点击坐标必须走同一个变换，否则框和鼠标对不上。
        """
        H, W = self.img.shape[:2]
        vw, vh = self.view_size
        k = self.s * self.zoom
        cw = min(float(W), vw / k)
        ch = min(float(H), vh / k)
        k = min(vw / cw, vh / ch)          # 横竖都塞满，保持比例
        cw, ch = vw / k, vh / k
        cx, cy = self.center
        x0 = min(max(0.0, cx - cw / 2), W - cw)
        y0 = min(max(0.0, cy - ch / 2), H - ch)
        return x0, y0, k, k, cw, ch

    @staticmethod
    def _union_box(boxes):
        """所有窗框并起来的外接框 ≈ 立面区域；一个框都没有就返回 None。"""
        if not boxes:
            return None
        x0 = min(b[0] for b in boxes)
        y0 = min(b[1] for b in boxes)
        x1 = max(b[0] + b[2] for b in boxes)
        y1 = max(b[1] + b[3] for b in boxes)
        return (x0, y0, x1, y1)

    def _in_facade(self, u, v, margin=0.6, min_pad=60):
        """这个点离"某个窗框"够近吗？够近才认为它落在那面墙上。

        为什么不用"所有窗框的外接框"：窗框散布整张图时，外接框≈整张图，
        点天空也会被判在立面内，于是照样显示同一个距离（用户实测发现的）。

        判据：把每个窗框各自外扩（框尺寸的 margin 倍，至少 min_pad 像素），
        点落在任意一个外扩框里，就算"在这面墙附近"。
        """
        for (x, y, w, h) in getattr(self, "win_boxes", []):
            px = max(min_pad, w * margin)
            py = max(min_pad, h * margin)
            if (x - px) <= u <= (x + w + px) and (y - py) <= v <= (y + h + py):
                return True
        return False

    def view_to_image(self, vx, vy):
        """视口坐标 → 原图坐标（点击/缩放都要用）。"""
        x0, y0, kx, ky, _, _ = self._tf()
        return x0 + vx / kx, y0 + vy / ky

    def zoom_at(self, factor, vx, vy):
        """以视口里的 (vx, vy) 为中心缩放：光标下的那个点保持不动。

        顺序很重要（踩过坑）：
            ① 先用【当前】zoom 求出光标下对应的是原图哪个点
            ② 再改 zoom
            ③ 最后反解中心，让那个点仍然落在 (vx, vy)
        如果先改 zoom 再求点，求出来的已经是"放大后该点跑到的位置"，
        算出的新中心≈旧中心，表现为"朝图像正中心放大"。
        """
        vw, vh = self.view_size
        u, v = self.view_to_image(vx, vy)          # ① 旧变换下取点
        old = self.zoom
        self.zoom = max(1.0, min(8.0, self.zoom * factor))
        if abs(self.zoom - old) < 1e-6:
            return
        k = self.s * self.zoom                      # ② 新缩放比
        self.center = (u + (vw / 2 - vx) / k,       # ③ 反解中心
                       v + (vh / 2 - vy) / k)

    # ---------- 绘制 ----------
    def render(self):
        """把当前状态画成一张图：绿框=窗户、黄框=红点候选、十字=选中。

        注意 dot_boxes 存的是 (x, y, w, h)，不是 (x0,y0,x1,y1)——
        早先按后者解包，框会从候选点一路拉到 (宽,高)，变成横跨半屏的巨框。
        """
        x0, y0, kx, ky, cw, ch = self._tf()
        vw, vh = self.view_size

        crop = self.img[int(y0):int(y0 + ch), int(x0):int(x0 + cw)]
        interp = cv2.INTER_AREA if kx < 1 else cv2.INTER_LANCZOS4
        view = cv2.resize(crop, (vw, vh), interpolation=interp)

        def PX(ix):
            return int((ix - x0) * kx)

        def PY(iy):
            return int((iy - y0) * ky)

        if self.show_win:
            for (x, y, w, h) in self.win_boxes:
                cv2.rectangle(view, (PX(x), PY(y)), (PX(x + w), PY(y + h)),
                              (110, 190, 110), 1)

        for k, (bx, by, bw, bh) in enumerate(self.dot_boxes):
            th = 3 if k == 0 else 1
            cv2.rectangle(view, (PX(bx), PY(by)), (PX(bx + bw), PY(by + bh)),
                          (0, 210, 255), th)
            if k < len(getattr(self, "dot_cands", [])):
                c = self.dot_cands[k]
                cv2.putText(view, f"R{k+1} cd{c.cdiff:+.2f} d{c.diameter_px:.0f}",
                            (PX(bx), max(12, PY(by) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 210, 255), 1)

        if self.sel:
            cu, cvv = self.sel["point"]
            px, py = PX(cu), PY(cvv)
            armed = self.sel["armed"]
            color = (0, 0, 255) if armed else (0, 255, 255)
            cv2.drawMarker(view, (px, py), color,
                           cv2.MARKER_CROSS if not armed else cv2.MARKER_TILTED_CROSS,
                           34 if armed else 24, 3 if armed else 2)
            cv2.circle(view, (px, py), 26, color, 1)
            cv2.circle(view, (px, py), 44, color, 1)

        top = f"{self.sel['cmd'].describe() if self.sel else '点击选定目标（双击锁定）'}"
        if self.sel and self.sel["armed"]:
            top = "★ " + top
        if self.zoom > 1.01:
            top += f"   [放大 {self.zoom:.1f}x]"
        cv2.rectangle(view, (0, 0), (view.shape[1], 36), (28, 28, 28), -1)
        cv2.putText(view, top, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.68,
                    (255, 255, 255), 2)

        foot = (f"[{self.files[self.idx % len(self.files)].name}] "
                f"{self.idx + 1}/{len(self.files)}   "
                f"绿框=窗户({len(self.win_boxes)})  黄框=红点候选({len(self.dot_cands)})   "
                f"单击=预选 双击=锁定 回车=发送 Esc=取消 n/p=切图 v=窗框 "
                f"滚轮=缩放 右键拖=平移")
        cv2.putText(view, foot, (10, view.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 230, 230), 1)
        if self.flash:
            cv2.putText(view, self.flash, (10, view.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (60, 220, 255), 2)
        if self.last_click:
            # 只要这行在变，就说明鼠标事件是通的
            cv2.putText(view, f"last click {self.last_click}",
                        (view.shape[1] - 240, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 220, 255), 2)
        return view


def main():
    files = [Path(sys.argv[1])] if len(sys.argv) > 1 else sorted(
        f for f in C.SAMPLES_DIR.iterdir() if f.suffix.lower() == ".jpg")
    demo = Demo(files)
    demo.load()
    print(f"Python: {sys.executable}")   # 一眼看出跑的是哪个环境（应为 mcpv）

    def on_mouse(event, x, y, flags, param):
        # 每次事件都打一行：这样"点了没反应"能立刻区分是
        #  (a) 事件压根没到窗口，还是 (b) 到了但内部抛了异常
        try:
            if event == cv2.EVENT_LBUTTONDOWN:
                print(f"[mouse] 左键 ({x},{y})")
                demo.select(x, y)
                demo.dirty = True
            elif event == cv2.EVENT_LBUTTONDBLCLK:
                print(f"[mouse] 双击 ({x},{y})")
                demo.arm()
                demo.dirty = True

            elif event == cv2.EVENT_MOUSEWHEEL:
                # 滚轮缩放（以光标为中心）：小框点不准时用这个放大
                d = wheel_delta(flags)
                if d:
                    demo.zoom_at(1.25 if d > 0 else 0.8, x, y)
                    print(f"[mouse] 滚轮 {d:+d} → 放大 {demo.zoom:.2f}x")

            elif event == cv2.EVENT_RBUTTONDOWN:
                demo.pan_anchor = (x, y)          # 开始右键拖拽

            elif event == cv2.EVENT_MOUSEMOVE and demo.pan_anchor is not None:
                ax, ay = demo.pan_anchor
                _, _, kx, ky, _, _ = demo._tf()
                # 画面跟着手走：往右拖 = 看更左边的内容
                demo.center = (demo.center[0] - (x - ax) / kx,
                               demo.center[1] - (y - ay) / ky)
                demo.pan_anchor = (x, y)
                demo.dirty = True

            elif event == cv2.EVENT_RBUTTONUP:
                demo.pan_anchor = None
        except Exception:
            import traceback
            print("[mouse] ★ 回调里出错（就是它让点击看起来没反应）：")
            traceback.print_exc()


    cv2.namedWindow("click-to-fly", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("click-to-fly", on_mouse)

    while True:
        # 只在状态变化时重绘：每帧都 resize 4096 宽的大图会挤掉鼠标消息
        if demo.dirty:
            cv2.imshow("click-to-fly", demo.render())
            demo.dirty = False
        k = cv2.waitKey(20) & 0xFF
        if k == ord("q"):            # 退出（等价于关窗口）
            break
        elif k in (13, 10):          # Enter = 发送
            demo.fire()
            demo.dirty = True
        elif k == 27:                # Esc = 取消选中
            demo.sel = None
            demo.dirty = True
        elif k == ord("n"):          # 下一张
            demo.idx += 1
            demo.load()
            demo.dirty = True
        elif k == ord("p"):          # 上一张
            demo.idx -= 1
            demo.load()
            demo.dirty = True
        elif k == ord("v"):          # v = view，开关窗户框
            # ★ 不要用 W：W/A/S/D 是文豪手动段的前后左右，演示中途按错就撞了
            demo.show_win = not demo.show_win
            demo.dirty = True
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
