# -*- coding: utf-8 -*-
"""验证用户的架构：VLM 给粗区域 → CV 在 ROI 内精检
在"100 m 等效"图像上测试

模拟方法：取 41 m 那帧（红点 7.6 px），整体缩小 41/100 = 0.41
→ 红点变 3.1 px，等价于 100 m 看到的像素尺寸。

对比：
  A 全图 严格阈值
  B 全图 宽松阈值
  C VLM 粗定位（整图 grounding）→ 框内 严格阈值
  D VLM 粗定位 → 框内 宽松阈值
  E 随机框（对照：证明是不是"只要框对了就行"）
"""
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
import mcpv.detect_live as D                                # noqa: E402
from mcpv.vlm import ask_crop                                # noqa: E402

VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
STRICT = dict(S_MIN=115, V_MIN=60, AREA_MIN=15, FILL_MIN=0.55,
              RD_MIN=50, RD_S_MIN=90, RD_AREA_MIN=6, RD_FILL_MIN=0.42)
LOOSE = dict(S_MIN=80, V_MIN=40, AREA_MIN=4, FILL_MIN=0.30,
             RD_MIN=26, RD_S_MIN=50, RD_AREA_MIN=2, RD_FILL_MIN=0.22)
FX = 1566.149


def set_thr(d):
    for k, v in d.items():
        setattr(D, k, v)


cap = cv2.VideoCapture(str(VID))
cap.set(cv2.CAP_PROP_POS_FRAMES, 60)      # 41 m 那帧
ok, fr41 = cap.read()
cap.release()

SC = 41.0 / 100.0
h, w = fr41.shape[:2]
fr100 = cv2.resize(fr41, (int(w * SC), int(h * SC)), interpolation=cv2.INTER_AREA)
# 记录红点在 100m 图里的真实位置（用 41m 帧的检出位置换算）
set_thr(STRICT)
c41 = D.detect_reds(fr41)
dot41 = min(c41, key=lambda q: abs(q["d"] - 7.6))
GT = (dot41["cx"] * SC, dot41["cy"] * SC)
print(f"41 m 帧：红点 d={dot41['d']:.1f}px @ ({dot41['cx']:.0f},{dot41['cy']:.0f})")
print(f"缩到 {SC:.2f}× → 图像 {fr100.shape[1]}x{fr100.shape[0]}，"
      f"红点约 {dot41['d']*SC:.1f} px，真值位置 {GT[0]:.0f},{GT[1]:.0f}")


def hit_in(cands, gt, tol=6.0):
    for q in cands:
        if np.hypot(q["cx"] - gt[0], q["cy"] - gt[1]) <= tol:
            return q
    return None


print("\n=== A 全图 严格阈值 ===")
set_thr(STRICT)
t0 = time.time(); ca = D.detect_reds(fr100); ta = (time.time()-t0)*1000
print(f"  候选 {len(ca)} 个  {ta:.1f} ms  命中：{hit_in(ca, GT) is not None}")

print("=== B 全图 宽松阈值 ===")
set_thr(LOOSE)
t0 = time.time(); cb = D.detect_reds(fr100); tb = (time.time()-t0)*1000
print(f"  候选 {len(cb)} 个  {tb:.1f} ms  命中：{hit_in(cb, GT) is not None}")

print("\n=== C/D VLM 粗定位 ===")
big = cv2.resize(fr100, (960, int(fr100.shape[0] * 960 / fr100.shape[1])))
q = ("Please provide the bounding box coordinate of the region this "
     "sentence describes: <ref>窗户</ref>")
t0 = time.time()
try:
    ans, _ = ask_crop(big, q, timeout=60)
    dtv = time.time() - t0
    print(f"  VLM 回答：{ans!r}  ({dtv:.1f}s)")
    import re
    nums = [int(x) for x in re.findall(r"\d+", ans or "")]
    if len(nums) >= 4:
        H100, W100 = fr100.shape[:2]
        x0 = nums[0] / 1000 * W100; y0 = nums[1] / 1000 * H100
        x1 = nums[2] / 1000 * W100; y1 = nums[3] / 1000 * H100
        print(f"  换算成 100m 图坐标：({x0:.0f},{y0:.0f})-({x1:.0f},{y1:.0f})"
              f"  真值 {GT[0]:.0f},{GT[1]:.0f}")
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        print(f"  框中心离真值 {np.hypot(cx-GT[0], cy-GT[1]):.0f} px")
        pad = max(40, int(max(x1 - x0, y1 - y0) * 0.5))
        rx0 = max(0, int(x0) - pad); ry0 = max(0, int(y0) - pad)
        rx1 = min(W100, int(x1) + pad); ry1 = min(H100, int(y1) + pad)
        roi = fr100[ry0:ry1, rx0:rx1]
        for name, thr in (("C 框内严格", STRICT), ("D 框内宽松", LOOSE)):
            set_thr(thr)
            t0 = time.time()
            cr = D.detect_reds(roi)
            ms = (time.time() - t0) * 1000
            g = [(x + rx0, y + ry0, d) for (x, y, d) in
                 [(q2["cx"], q2["cy"], q2["d"]) for q2 in cr]]
            found = any(np.hypot(gx - GT[0], gy - GT[1]) <= 6 for gx, gy, _ in g)
            print(f"  {name}: ROI {rx1-rx0}x{ry1-ry0}  候选 {len(cr)}  "
                  f"{ms:.0f} ms  命中：{found}")
        # 对照 E：用真值位置开同样大小的框
        rx0b = max(0, int(GT[0] - (rx1 - rx0) / 2)); ry0b = max(0, int(GT[1] - (ry1 - ry0) / 2))
        roi2 = fr100[ry0b:ry0b + (ry1 - ry0), rx0b:rx0b + (rx1 - rx0)]
        set_thr(STRICT)
        cr2 = D.detect_reds(roi2)
        g2 = [(q2["cx"] + rx0b, q2["cy"] + ry0b) for q2 in cr2]
        found2 = any(np.hypot(gx - GT[0], gy - GT[1]) <= 6 for gx, gy in g2)
        print(f"  E 对照（以真值为中心同尺寸框，严格）: 命中：{found2}")
    else:
        print("  VLM 没给出可用框")
except Exception as e:
    print(f"  VLM 失败 {type(e).__name__}: {e}")
