# -*- coding: utf-8 -*-
"""
双目 ArUco 测距（读取 stereo_calib.py 生成的 stereo_calib.yaml）
和 stereo_calib.py 放在同一个文件夹。

原理：左右眼分别检测 ArUco 的 4 个角点 -> 用标定参数校正到同一行 -> 视差三角化出 3D 坐标。
     不需要知道码的尺寸；脚本反算出的码边长(side)可以用尺子核对，用来验证尺度。

用法：
    python stereo_range.py                     # 默认 DICT_4X4_50，摄像头 1
    python stereo_range.py --marker-mm 186     # 给出码的实测边长(外黑框)，额外显示单目 PnP 距离对比
    python stereo_range.py --ids 0 3           # 只测指定 ID（标定板也是 4x4 码，别让它进画面或用这个过滤）
    python stereo_range.py --disp-offset 0.35  # 用实测数据拟合出的视差偏置修正远距离（退出时会给出建议值）
    python stereo_range.py --exposure -8       # 手动曝光 1/256s，减少运动模糊（-9 = 1/512s，越小越暗）
    检测前会自动做软件提亮（画面暗时把亮度拉起来，亮时不变），短曝光下也能检测

按键：
    r  记录当前读数，终端里输入卷尺实测距离，自动算误差并写入 ranging_log.csv
    c  清空平滑历史
    p  保存当前原始画面 snap_XXX.png（检测不到时可以发给别人分析）
    q  退出（会打印本次记录的汇总）

画面第一行：近 2 秒左右眼各自的检出率和被拒候选数
    检出率低 + 被拒候选多 -> 找到了方框但读不出码（模糊/反光/曝光），加光、减少晃动
    检出率低 + 被拒候选少 -> 连方框都没找到（太小/太暗/对比度低）
短暂丢失（<0.5s）时继续显示上一次读数（灰色，标 held）。
"""
import argparse
import collections
import csv
import os
import sys
import time

import cv2
import numpy as np

try:
    from stereo_calib import open_cam, split, preview_scale, setup_window, hud, load_calib, WIN
except ImportError:
    sys.exit("请把 stereo_range.py 和 stereo_calib.py 放在同一个文件夹里")

HIST = 15            # 平滑窗口（帧）
HOLD = 15            # 丢失后保持显示的帧数（约 0.5s）
RATE_WIN = 60        # 检出率统计窗口（帧，约 2s）


def brighten(g):
    """软件提亮：把较亮的像素拉到 ~220。画面本来够亮时系数为 1（不变）。
    只是线性缩放，不改变角点位置；让 ArUco 的自适应阈值在暗画面里重新起作用
    （仿真：1/1024s 级别的暗画面，不提亮检出率 0%，提亮后 100%）"""
    p = float(np.percentile(g[::4, ::4], 99.5))
    a = float(np.clip(220.0 / max(p, 1.0), 1.0, 6.0))
    return (g if a == 1.0 else cv2.convertScaleAbs(g, alpha=a)), a


class PairDetector:
    """左右眼一起检测。优先 CONTOUR 精化（远处抖动小）；OpenCV 5.0 的 CONTOUR 在码模糊/残缺时
    会断言崩溃（nContours.size() >= 2），出错的那一帧左右眼都改用 SUBPIX，保证两眼方法一致。"""

    def __init__(self, dict_name):
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

        def mk(method):
            p = cv2.aruco.DetectorParameters()
            p.cornerRefinementMethod = method
            p.minMarkerPerimeterRate = 0.01      # 允许更小（更远）的码
            # 每格多采样、忽略格子边缘：运动模糊/暗光下远距离检出率大幅提高，且不增加误检
            # （仿真：7m 85%->100%，9m 15%->98%；放宽阈值/多边形/纠错位反而误检暴增，不用）
            p.perspectiveRemovePixelPerCell = 8
            p.perspectiveRemoveIgnoredMarginPerCell = 0.25
            return cv2.aruco.ArucoDetector(d, p)

        self.main = mk(cv2.aruco.CORNER_REFINE_CONTOUR)
        self.fallback = mk(cv2.aruco.CORNER_REFINE_SUBPIX)
        self.fallbacks = 0

    def __call__(self, gl, gr):
        try:
            a = self.main.detectMarkers(gl)
            b = self.main.detectMarkers(gr)
        except cv2.error:
            self.fallbacks += 1
            a = self.fallback.detectMarkers(gl)
            b = self.fallback.detectMarkers(gr)
        return (a[0], a[1], len(a[2])), (b[0], b[1], len(b[2]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default="stereo_calib.yaml")
    ap.add_argument("--cam", type=int, default=1)
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--marker-mm", type=float, default=0, help="码外黑框实测边长 mm，用于单目 PnP 对比（可选）")
    ap.add_argument("--ids", type=int, nargs="*", default=None, help="只测这些 ID")
    ap.add_argument("--preview", type=float, default=0)
    ap.add_argument("--log", default="ranging_log.csv")
    ap.add_argument("--exposure", type=float, default=None,
                    help="手动曝光：-6=1/64s, -7=1/128s, -8=1/256s, -9=1/512s；默认自动曝光")
    ap.add_argument("--disp-offset", type=float, default=0.0,
                    help="视差偏置 px：Z = f*B/(d - offset)。用卷尺数据拟合，退出时会打印建议值")
    args = ap.parse_args()

    if not os.path.exists(args.calib):
        sys.exit(f"找不到 {args.calib}，先运行 stereo_calib.py 标定")
    c = load_calib(args.calib)
    K1, D1, K2, D2 = c["K1"], c["D1"], c["K2"], c["D2"]
    R1, R2, P1, P2 = c["R1"], c["R2"], c["P1"], c["P2"]
    f, cx, cy = P1[0, 0], P1[0, 2], P1[1, 2]
    B = -P2[0, 3] / P2[0, 0]                  # 基线 mm
    Kr = P1[:3, :3]                           # 校正后左眼内参（无畸变）
    d0 = args.disp_offset
    unit = np.array([[-.5, .5, 0], [.5, .5, 0], [.5, -.5, 0], [-.5, -.5, 0]], np.float32)
    print(f"标定：f={f:.1f}px  基线={B:.2f}mm  swap={int(c['swap'])} flip={int(c['flip'])}  视差偏置={d0}px")

    det = PairDetector(args.dict)
    s = args.marker_mm
    obj = (np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float32)
           if s > 0 else None)

    hist = collections.defaultdict(lambda: collections.deque(maxlen=HIST))
    last_seen = {}
    seenL = collections.deque(maxlen=RATE_WIN)
    seenR = collections.deque(maxlen=RATE_WIN)
    nframe = 0

    def want(ids):
        return ids is not None and any((not args.ids) or int(i) in args.ids for i in ids.flatten())
    records = []
    scale = preview_scale(args.preview)
    cap = open_cam(args.cam, args.exposure)
    setup_window(scale)
    print("r 记录（终端输入卷尺距离）   c 清空平滑   p 保存画面   q 退出")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读帧失败")
                break
            L, R = split(frame, c["swap"], c["flip"])
            gl, al = brighten(cv2.cvtColor(L, cv2.COLOR_BGR2GRAY))
            gr, ar = brighten(cv2.cvtColor(R, cv2.COLOR_BGR2GRAY))
            (cl, il, rejL), (cr, ir, rejR) = det(gl, gr)
            nframe += 1
            seenL.append(want(il))
            seenR.append(want(ir))
            nl = 0 if il is None else len(il)
            nr = 0 if ir is None else len(ir)

            vl, vr = L.copy(), R.copy()
            if nl:
                cv2.aruco.drawDetectedMarkers(vl, cl, il)
            if nr:
                cv2.aruco.drawDetectedMarkers(vr, cr, ir)

            meas = {}
            if nl and nr:
                right = {int(i): q.reshape(4, 2) for i, q in zip(ir.flatten(), cr)}
                for q, i in zip(cl, il.flatten()):
                    i = int(i)
                    if i not in right or (args.ids and i not in args.ids):
                        continue
                    pl, pr = q.reshape(4, 2), right[i]
                    # 校正到同一行（不重映射整幅图，直接变换角点坐标，更快也更准）
                    ul = cv2.undistortPoints(pl.reshape(-1, 1, 2), K1, D1, R=R1, P=P1).reshape(4, 2)
                    ur = cv2.undistortPoints(pr.reshape(-1, 1, 2), K2, D2, R=R2, P=P2).reshape(4, 2)
                    disp = ul[:, 0] - ur[:, 0]
                    dy = float(np.abs(ul[:, 1] - ur[:, 1]).mean())
                    dm = float(disp.mean())                            # 4 个角点平均视差，比逐点更稳
                    if (disp <= 0).any() or dm - d0 <= 0:
                        meas[i] = dict(bad="disparity <= 0 (L/R swapped or mismatch)", pl=pl)
                        continue
                    Zc = f * B / (dm - d0)                             # mm
                    xm, ym = ul[:, 0].mean(), (ul[:, 1].mean() + ur[:, 1].mean()) / 2
                    ctr = np.array([(xm - cx) * Zc / f, (ym - cy) * Zc / f, Zc])
                    # 码边长：单目 PnP 给出形状和姿态（单位边长），双目给出尺度
                    _, _, tu = cv2.solvePnP(unit, ul.astype(np.float32), Kr, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    side = float(np.linalg.norm(ctr) / np.linalg.norm(tu))
                    pnp = None
                    if obj is not None:
                        okp, rv, tv = cv2.solvePnP(obj, pl, K1, D1, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        if okp:
                            pnp = float(np.linalg.norm(tv)) / 1000
                    m = dict(dist=float(np.linalg.norm(ctr)) / 1000, depth=float(ctr[2]) / 1000,
                             disp=dm, dy=dy, side=side, pnp=pnp, pl=pl,
                             px=float(np.mean([np.linalg.norm(pl[k] - pl[(k + 1) % 4]) for k in range(4)])))
                    hist[i].append(m)
                    meas[i] = m
                    last_seen[i] = nframe

            # ---- 显示
            rl, rr = np.mean(seenL), np.mean(seenR)
            rb = np.mean([a and b for a, b in zip(seenL, seenR)])
            rcol = (0, 255, 0) if rb > 0.95 else ((0, 200, 255) if rb > 0.7 else (0, 0, 255))
            lines = [(f"detect (2s): L {rl:.0%}  R {rr:.0%}  both {rb:.0%}   rejected L {rejL} R {rejR}"
                      f"   sw gain x{max(al, ar):.1f}", rcol)]
            k = vl.shape[1] * 2 * scale / 1536.0
            for i, m in sorted(meas.items()):
                tl = tuple(int(v) for v in m["pl"].min(0))
                if "bad" in m:
                    lines.append((f"id {i}: {m['bad']}", (0, 0, 255)))
                    continue
                h = hist[i]
                dist = np.median([x["dist"] for x in h])
                depth = np.median([x["depth"] for x in h])
                side = np.median([x["side"] for x in h])
                jit = np.std([x["dist"] for x in h]) * 100 if len(h) > 3 else 0
                col = (0, 255, 0) if m["dy"] < 1.5 else (0, 165, 255)
                t = (f"id {i}: dist {dist:.3f} m  depth {depth:.3f} m  jitter {jit:.1f} cm  "
                     f"disp {m['disp']:.1f}px  dy {m['dy']:.2f}px  side {side:.0f} mm  ({m['px']:.0f}px)")
                if m["pnp"] is not None:
                    t += f"  PnP {np.median([x['pnp'] for x in h if x['pnp']]):.3f} m"
                lines.append((t, col))
                cv2.putText(vl, f"{dist:.2f} m", (tl[0], max(30, tl[1] - 15)), cv2.FONT_HERSHEY_SIMPLEX,
                            max(1.0, 0.8 / scale / 2), (0, 255, 255), max(2, int(1 / scale)), cv2.LINE_AA)
            for i in sorted(last_seen):
                lost = nframe - last_seen[i]
                if i in meas or lost > HOLD or not hist[i]:
                    continue
                h = hist[i]
                lines.append((f"id {i}: dist {np.median([x['dist'] for x in h]):.3f} m  (held, lost {lost} frames)",
                              (170, 170, 170)))
            if len(lines) == 1:
                lines.append((f"no marker seen in BOTH eyes   (L {nl}, R {nr})", (0, 0, 255)))
            pv = cv2.resize(cv2.hconcat([vl, vr]), None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            hud(pv, lines, "r record (type tape distance in terminal) | c clear | p snapshot | q quit")
            cv2.imshow(WIN, pv)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                hist.clear()
                last_seen.clear()
            elif key == ord('p'):
                k_ = 1
                while os.path.exists(f"snap_{k_:03d}.png"):
                    k_ += 1
                cv2.imwrite(f"snap_{k_:03d}.png", frame)
                print(f"已保存原始画面 snap_{k_:03d}.png（左右拼接 3840x1080）")
            elif key == ord('r'):
                good = {i: m for i, m in meas.items() if "bad" not in m}
                if not good:
                    print("当前没有左右都看到的码")
                    continue
                for i in good:
                    h = hist[i]
                    dist = float(np.median([x["dist"] for x in h]))
                    depth = float(np.median([x["depth"] for x in h]))
                    side = float(np.median([x["side"] for x in h]))
                    pnps = [x["pnp"] for x in h if x["pnp"]]
                    pnp = float(np.median(pnps)) if pnps else None
                    print(f"\nid {i}: 双目距离 {dist:.3f} m（深度 {depth:.3f} m），码边长 {side:.0f} mm"
                          + (f"，PnP {pnp:.3f} m" if pnp else ""))
                    if args.marker_mm > 0 and abs(side / args.marker_mm - 1) > 0.08:
                        print(f"  提示：双目测出码边长 {side:.0f} mm，与 --marker-mm {args.marker_mm:g} 不符，"
                              f"请用尺子量外黑框；PnP 距离会按比例偏差")
                    s_in = input("  卷尺实测距离 (m，直接回车跳过)：").strip()
                    truth = float(s_in) if s_in else None
                    dmed = float(np.median([x["disp"] for x in h]))
                    row = dict(time=time.strftime("%H:%M:%S"), id=i, truth_m=truth,
                               stereo_dist_m=round(dist, 4), stereo_depth_m=round(depth, 4),
                               side_mm=round(side, 1), pnp_m=None if pnp is None else round(pnp, 4),
                               disp_px=round(dmed, 3), dy_px=round(good[i]["dy"], 2),
                               _ratio=dist / depth)
                    if truth:
                        e = dist - truth
                        row.update(err_cm=round(e * 100, 1), err_pct=round(e / truth * 100, 2))
                        print(f"  误差 {e * 100:+.1f} cm（{e / truth * 100:+.2f}%）")
                    records.append(row)
                    new = not os.path.exists(args.log)
                    with open(args.log, "a", newline="", encoding="utf-8-sig") as fp:
                        w = csv.DictWriter(fp, fieldnames=["time", "id", "truth_m", "stereo_dist_m",
                                                           "stereo_depth_m", "err_cm", "err_pct", "side_mm",
                                                           "pnp_m", "disp_px", "dy_px"], extrasaction="ignore")
                        if new:
                            w.writeheader()
                        w.writerow(row)
                hist.clear()
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if records:
        print("\n===== 本次记录 =====")
        print(f"{'实测m':>7} {'双目m':>8} {'误差cm':>8} {'误差%':>7} {'边长mm':>7} {'PnP m':>7}")
        for r in records:
            print(f"{r['truth_m'] or '-':>7} {r['stereo_dist_m']:>8.3f} {r.get('err_cm', '-'):>8} "
                  f"{r.get('err_pct', '-'):>7} {r['side_mm']:>7.0f} {r['pnp_m'] or '-':>7}")
        print(f"已追加到 {args.log}")

        # 拟合视差偏置：真实视差 = f*B/Z_true，偏置 = 测得视差 - 真实视差（应对所有距离近似为常数）
        fit = [r for r in records if r["truth_m"]]
        if len(fit) >= 2:
            off = [r["disp_px"] - f * B / (r["truth_m"] / r["_ratio"] * 1000) for r in fit]
            tot = float(np.median(off))              # 记录的是原始视差，所以这就是总偏置
            print(f"\n各点视差偏差: {', '.join(f'{o:+.2f}' for o in off)} px")
            print(f"建议视差偏置 --disp-offset {tot:.2f}   （当前 {d0}）")
            print("修正后误差：", ", ".join(
                f"{r['truth_m']}m→{(f * B / (r['disp_px'] - tot) * r['_ratio'] / 1000 - r['truth_m']) * 100:+.1f}cm"
                for r in fit))
            if np.std(off) > 0.3:
                print("注意：各距离的偏差不一致（>0.3px），可能是量距不准或码没对正，偏置修正效果有限")


if __name__ == "__main__":
    main()
