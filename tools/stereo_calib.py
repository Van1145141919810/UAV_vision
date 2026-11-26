# -*- coding: utf-8 -*-
"""
USB 双目相机（3840x1080 拼图，每眼 1920x1080）ChArUco 标定工具
需要 OpenCV >= 4.7

最常用：直接运行，一个窗口走完全流程
    python stereo_calib.py                 # 摄像头默认索引 1，可加 --cam 0
    python stereo_calib.py --flip          # 相机倒装时（标定和使用必须同一朝向）
    python stereo_calib.py --square 27.9   # 方格实测不是 28mm 时
    python stereo_calib.py --preview 0.3   # 手动指定预览缩放（默认按屏幕宽度自动）

  窗口按键：
    s  保存当前这对图（顶部显示绿色 READY 时才能存）
    d  删除上一对（拍坏了就删）
    w  交换左右眼（画面提示 L/R swapped 时按）
    c  用已保存的图标定，成功后自动切到校正画面
    v  在 采集画面 / 校正画面 之间切换（需已标定）
    q  退出
  窗口可以用鼠标拖动改变大小。

其他子命令：
    python stereo_calib.py board            # 生成 A4 标定板 PDF
    python stereo_calib.py calib            # 只对已拍的图做标定
    python stereo_calib.py view             # 只看校正画面
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

DICT_ID = cv2.aruco.DICT_4X4_50
COLS, ROWS = 9, 6            # 方格数（不是内角点数）
MARKER_RATIO = 0.75          # marker 边长 / 方格边长
EYE_W, EYE_H = 1920, 1080
MIN_CORNERS = 12             # 左右共同角点至少多少个才保存/使用
STILL_PX = 1.5               # 相邻两帧角点平均移动小于此值（像素）才算静止
WIN = "stereo calib"


# ---------------------------------------------------------------- 公共
def make_board(square_mm):
    d = cv2.aruco.getPredefinedDictionary(DICT_ID)
    return cv2.aruco.CharucoBoard((COLS, ROWS), square_mm, square_mm * MARKER_RATIO, d)


def make_detector(board):
    cp = cv2.aruco.CharucoParameters()
    dp = cv2.aruco.DetectorParameters()
    dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.CharucoDetector(board, cp, dp)


def detect(detector, img):
    """返回 (corners Nx2 float32, ids N int) 或 (None, None)"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    cc, ci, _, _ = detector.detectBoard(gray)
    if ci is None or len(ci) < 4:
        return None, None
    return cc.reshape(-1, 2).astype(np.float32), ci.flatten().astype(int)


def well_spread(ids):
    """角点不能全在一行或一列上，否则标定退化"""
    ids = np.asarray(ids)
    rows, cols = ids // (COLS - 1), ids % (COLS - 1)
    return len(set(rows.tolist())) >= 2 and len(set(cols.tolist())) >= 2


def open_cam(idx, exposure=None):
    """exposure: 手动曝光值。Windows MSMF 下是 log2(秒)，-6=1/64s，-8=1/256s，-9=1/512s；
    这台相机读回值一直是 -6，但实际有效（看画面亮度）。None = 保持自动曝光"""
    backend = cv2.CAP_MSMF if sys.platform.startswith("win") else cv2.CAP_V4L2
    cap = cv2.VideoCapture(idx, backend)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, EYE_W * 2)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, EYE_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (w, h) != (EYE_W * 2, EYE_H):
        raise SystemExit(f"分辨率是 {w}x{h}，不是 3840x1080：检查 --cam 索引")
    if exposure is not None:
        if backend == cv2.CAP_V4L2:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)          # V4L2: 1 = manual，曝光单位 100µs
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)       # 0.25 = manual
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        print(f"手动曝光 {exposure:g}" + (f"（约 1/{2 ** -exposure:.0f} s）" if backend != cv2.CAP_V4L2 and exposure < 0 else ""))
    return cap


def split(frame, swap, flip=False):
    """返回 (左眼, 右眼)。flip: 相机倒装时先把整帧转 180°，左右关系不变"""
    if flip:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    a, b = frame[:, :EYE_W], frame[:, EYE_W:]
    return (b, a) if swap else (a, b)


# ---------------------------------------------------------------- 显示
def preview_scale(arg):
    """预览缩放：手动指定优先；否则按屏幕宽度让左右两眼刚好放下"""
    if arg and arg > 0:
        return arg
    sw = None
    if sys.platform.startswith("win"):
        try:
            import ctypes
            sw = ctypes.windll.user32.GetSystemMetrics(0)   # 考虑了系统缩放后的宽度
        except Exception:
            sw = None
    if not sw:
        return 0.33
    return float(np.clip(sw * 0.92 / (2 * EYE_W), 0.15, 0.5))


def setup_window(scale):
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(WIN, int(2 * EYE_W * scale), int(EYE_H * scale))


def hud(img, lines, bottom=None):
    """在预览图上写字，字号随预览宽度变化。lines: [(text, color), ...]"""
    k = img.shape[1] / 1536.0
    fs, lh, th = 0.75 * k, int(32 * k), max(1, round(2 * k))

    def label(t, x, y, col, s):
        (w, h), base = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, s, th)
        pad = max(2, int(4 * k))
        cv2.rectangle(img, (x - pad, y - h - pad), (x + w + pad, y + base + pad), (0, 0, 0), -1)
        cv2.putText(img, t, (x, y), cv2.FONT_HERSHEY_SIMPLEX, s, col, th, cv2.LINE_AA)

    for n, (t, col) in enumerate(lines):
        label(t, int(10 * k), int(28 * k) + n * lh, col, fs)
    if bottom:
        label(bottom, int(10 * k), img.shape[0] - int(14 * k), (255, 255, 255), fs * 0.85)
    return img


# ---------------------------------------------------------------- board
def cmd_board(args):
    from PIL import Image, ImageDraw, ImageFont
    PPM = 20                                   # 20 px/mm = 508 dpi
    page_w, page_h = 297 * PPM, 210 * PPM      # A4 横向
    sq = args.square
    bw, bh = int(round(COLS * sq * PPM)), int(round(ROWS * sq * PPM))
    img = make_board(sq).generateImage((bw, bh), marginSize=0, borderBits=1)

    page = Image.new("L", (page_w, page_h), 255)
    x0, y0 = (page_w - bw) // 2, 10 * PPM
    page.paste(Image.fromarray(img), (x0, y0))
    d = ImageDraw.Draw(page)
    ry = y0 + bh + 9 * PPM
    d.line([(x0, ry), (x0 + 100 * PPM, ry)], fill=0, width=4)
    for mm in range(101):
        L = 5 if mm % 10 == 0 else (3 if mm % 5 == 0 else 1.5)
        d.line([(x0 + mm * PPM, ry), (x0 + mm * PPM, ry - L * PPM)], fill=0, width=3)
    font = ImageFont.load_default(size=int(3.5 * PPM))
    d.text((x0 + 104 * PPM, ry - 4.5 * PPM), "100 mm", fill=0, font=font)
    info = (f"ChArUco {COLS}x{ROWS} squares, DICT_4X4_50, square {sq:g} mm, marker {sq * MARKER_RATIO:g} mm, "
            f"board {COLS * sq:g} x {ROWS * sq:g} mm.  PRINT AT 100% / ACTUAL SIZE (no 'fit to page').")
    d.text((x0, ry + 3 * PPM), info, fill=0, font=ImageFont.load_default(size=int(2.8 * PPM)))
    page.save(args.out, resolution=PPM * 25.4)
    print(f"已生成 {args.out}：方格 {sq} mm，整板 {COLS * sq:g} x {ROWS * sq:g} mm")


# ---------------------------------------------------------------- calib
def view_errors(obj, img, rvecs, tvecs, K, D):
    errs = []
    for o, p, r, t in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, D)
        errs.append(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - p) ** 2, 1))))
    return np.array(errs)


def mono_calib(name, obj, img, names, size, flags):
    rms, K, D, rv, tv = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    e = view_errors(obj, img, rv, tv, K, D)
    print(f"[{name}] {len(obj)} 张  RMS = {rms:.3f}px  fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} "
          f"cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}  D={np.round(D.ravel(), 4)}")
    worst = np.argsort(-e)[:3]
    print("     误差最大的图: " + ", ".join(f"{names[k]}({e[k]:.2f}px)" for k in worst))
    return K, D


def run_calib(square, img_dir, swap, flip, out, k3=False):
    """标定，成功返回 True；失败打印原因返回 False"""
    board = make_board(square)
    det = make_detector(board)
    P3 = board.getChessboardCorners().astype(np.float32)   # (N,3)，单位 mm
    files = sorted(glob.glob(os.path.join(img_dir, "left", "*.png")))
    if not files:
        print(f"{img_dir}/left 下没有图")
        return False

    mono = {"L": ([], [], []), "R": ([], [], [])}
    st_obj, st_l, st_r, st_names = [], [], [], []
    size = None
    for fl in files:
        name = os.path.basename(fl)
        imL, imR = cv2.imread(fl), cv2.imread(os.path.join(img_dir, "right", name))
        if imL is None or imR is None:
            continue
        size = (imL.shape[1], imL.shape[0])
        res = {}
        for eye, im in (("L", imL), ("R", imR)):
            c, i = detect(det, im)
            if c is not None and len(i) >= MIN_CORNERS and well_spread(i):
                mono[eye][0].append(P3[i])
                mono[eye][1].append(c)
                mono[eye][2].append(name)
                res[eye] = dict(zip(i.tolist(), c))
        if "L" in res and "R" in res:
            common = sorted(set(res["L"]) & set(res["R"]))
            if len(common) >= MIN_CORNERS and well_spread(common):
                st_obj.append(P3[common])
                st_l.append(np.array([res["L"][k] for k in common], np.float32))
                st_r.append(np.array([res["R"][k] for k in common], np.float32))
                st_names.append(name)
    print(f"\n共 {len(files)} 对图，可用于双目的 {len(st_obj)} 对")
    if len(st_obj) < 10:
        print("可用图对太少（<10），再多拍一些")
        return False

    flags = 0 if k3 else cv2.CALIB_FIX_K3
    K1, D1 = mono_calib("左眼", *mono["L"], size, flags)
    K2, D2 = mono_calib("右眼", *mono["R"], size, flags)

    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)
    rms, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        st_obj, st_l, st_r, K1, D1, K2, D2, size, criteria=crit, flags=cv2.CALIB_FIX_INTRINSIC)
    B = float(np.linalg.norm(T))
    ang = np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0]))
    print(f"[双目] RMS = {rms:.3f}px   基线 |T| = {B:.2f} mm   T = {np.round(T.ravel(), 2)}   两眼夹角 {ang:.2f}°")

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    dys = []
    for pl, pr in zip(st_l, st_r):
        ul = cv2.undistortPoints(pl.reshape(-1, 1, 2), K1, D1, R=R1, P=P1).reshape(-1, 2)
        ur = cv2.undistortPoints(pr.reshape(-1, 1, 2), K2, D2, R=R2, P=P2).reshape(-1, 2)
        dys.append(ul[:, 1] - ur[:, 1])
    dys = np.abs(np.concatenate(dys))
    print(f"[校正] 左右行差 平均 {dys.mean():.3f}px  95% {np.percentile(dys, 95):.3f}px  最大 {dys.max():.3f}px")

    f = P1[0, 0]
    print(f"[精度] 校正后 f = {f:.1f}px，按 0.25px 视差误差估算：")
    for Z in (1, 2, 3, 5, 8):
        print(f"     {Z} m: 视差 {f * B / (Z * 1000):6.1f}px   "
              f"深度误差约 ±{(Z * 1000) ** 2 / (f * B) * 0.25 / 10:.1f} cm")

    verdict = []
    if T[0, 0] > 0:
        verdict.append("T 的 x 分量为正，左右眼喂反了：删掉图片，按 w 切换后重拍")
    if rms > 1.0:
        verdict.append("双目 RMS 偏大，删掉误差最大的图后再按 c")
    if not 50 < B < 70:
        verdict.append("基线偏离 60mm 较多，先检查 --square 方格尺寸")
    if dys.mean() > 0.5:
        verdict.append("校正行差偏大，补拍不同距离/倾斜的图")
    print("[结论] " + ("看起来合格" if not verdict else "；".join(verdict)))

    fs = cv2.FileStorage(out, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", size[0]); fs.write("image_height", size[1])
    fs.write("square_mm", square)
    fs.write("swap_halves", int(swap)); fs.write("flip_180", int(flip))
    for k, v in dict(K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T, E=E, F=F,
                     R1=R1, R2=R2, P1=P1, P2=P2, Q=Q).items():
        fs.write(k, v)
    fs.write("rms_stereo", rms)
    fs.release()
    print(f"已保存 {out}（swap={int(swap)} flip={int(flip)}，以后取图必须用同样的设置）")

    mL, mR = make_maps(load_calib(out))
    name = st_names[len(st_names) // 2]
    rl = cv2.remap(cv2.imread(os.path.join(img_dir, "left", name)), *mL, cv2.INTER_LINEAR)
    rr = cv2.remap(cv2.imread(os.path.join(img_dir, "right", name)), *mR, cv2.INTER_LINEAR)
    cv2.imwrite("rectify_check.jpg", cv2.resize(draw_lines(cv2.hconcat([rl, rr])), None, fx=0.5, fy=0.5))
    print("校正预览已存为 rectify_check.jpg\n")
    return True


def load_calib(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    c = {k: fs.getNode(k).mat() for k in ("K1", "D1", "K2", "D2", "R1", "R2", "P1", "P2", "Q")}
    c["size"] = (int(fs.getNode("image_width").real()), int(fs.getNode("image_height").real()))
    c["swap"] = bool(int(fs.getNode("swap_halves").real()))
    c["flip"] = bool(int(fs.getNode("flip_180").real())) if not fs.getNode("flip_180").empty() else False
    fs.release()
    return c


def make_maps(c):
    mL = cv2.initUndistortRectifyMap(c["K1"], c["D1"], c["R1"], c["P1"], c["size"], cv2.CV_16SC2)
    mR = cv2.initUndistortRectifyMap(c["K2"], c["D2"], c["R2"], c["P2"], c["size"], cv2.CV_16SC2)
    return mL, mR


def draw_lines(pair, step=40):
    for y in range(0, pair.shape[0], step):
        cv2.line(pair, (0, y), (pair.shape[1], y), (0, 255, 0), 1)
    return pair


def rectified_preview(L, R, maps, scale):
    mL, mR = maps
    pair = cv2.hconcat([cv2.remap(L, *mL, cv2.INTER_LINEAR), cv2.remap(R, *mR, cv2.INTER_LINEAR)])
    pv = cv2.resize(pair, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return draw_lines(pv, 16)          # 缩放后再画线，线不会被缩没


def check_setting(c, args):
    if c["swap"] != args.swap or c["flip"] != args.flip:
        print(f"注意：标定文件用的是 swap={int(c['swap'])} flip={int(c['flip'])}，"
              f"当前是 swap={int(args.swap)} flip={int(args.flip)}，校正画面会不对")


# ---------------------------------------------------------------- live（采集 + 标定 + 查看）
def cmd_live(args):
    board = make_board(args.square)
    det = make_detector(board)
    ldir, rdir = os.path.join(args.dir, "left"), os.path.join(args.dir, "right")
    os.makedirs(ldir, exist_ok=True)
    os.makedirs(rdir, exist_ok=True)
    scale = preview_scale(args.preview)

    # 已有的图：接着编号，并把它们的角点画进覆盖图
    saved = sorted(os.path.basename(p) for p in glob.glob(os.path.join(ldir, "*.png")))
    coverage = {}
    for n in saved:
        im = cv2.imread(os.path.join(ldir, n))
        c, _ = detect(det, im) if im is not None else (None, None)
        coverage[n] = [] if c is None else c.tolist()
    if saved:
        print(f"{args.dir} 里已有 {len(saved)} 对图，接着拍（要从头拍就先删掉这个文件夹）")

    maps = None
    if os.path.exists(args.calib):
        c = load_calib(args.calib)
        check_setting(c, args)
        maps = make_maps(c)
    mode = "capture"
    prev = None
    cap = open_cam(args.cam, args.exposure)
    setup_window(scale)
    print(f"预览缩放 {scale:.2f}（窗口可拖动改大小）")
    print("s 保存   d 删除上一对   w 交换左右   c 标定   v 切换校正画面   q 退出")
    keys_help = "s save | d delete last | w swap L/R | c calibrate | v rectified | q quit"
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读帧失败")
                break
            L, R = split(frame, args.swap, args.flip)
            ready = enough = swapped = False

            if mode == "rect":
                pv = rectified_preview(L, R, maps, scale)
                hud(pv, [("RECTIFIED: same point should sit on the same green line", (0, 255, 0))],
                    "v back to capture | q quit")
            else:
                cl, il = detect(det, L)
                cr, ir = detect(det, R)
                common = (np.intersect1d(il, ir) if il is not None and ir is not None
                          else np.array([], int))

                # 左右检查：板子在有限距离，左眼中的 x 应大于右眼中的 x（视差为正）
                if len(common) >= 4:
                    dl = dict(zip(il.tolist(), cl)); dr = dict(zip(ir.tolist(), cr))
                    dx = np.median([dl[k][0] - dr[k][0] for k in common.tolist()])
                    swapped = dx < 0

                # 静止判断：同一 ID 的角点在相邻两帧之间的平均移动
                cur = {} if il is None else dict(zip(il.tolist(), cl))
                still = False
                if prev:
                    ks = [k for k in cur if k in prev]
                    if len(ks) >= 8:
                        still = np.mean([np.linalg.norm(cur[k] - prev[k]) for k in ks]) < STILL_PX
                prev = cur

                enough = len(common) >= MIN_CORNERS and well_spread(common)
                ready = enough and still and not swapped

                vl, vr = L.copy(), R.copy()
                for pts in coverage.values():
                    for p in pts:
                        cv2.circle(vl, (int(p[0]), int(p[1])), 7, (0, 200, 0), -1)
                if il is not None:
                    cv2.aruco.drawDetectedCornersCharuco(vl, cl.reshape(-1, 1, 2), il.reshape(-1, 1), (0, 0, 255))
                if ir is not None:
                    cv2.aruco.drawDetectedCornersCharuco(vr, cr.reshape(-1, 1, 2), ir.reshape(-1, 1), (0, 0, 255))
                cv2.line(vl, (EYE_W - 2, 0), (EYE_W - 2, EYE_H), (255, 255, 255), 4)   # 左右分隔线
                pv = cv2.resize(cv2.hconcat([vl, vr]), None, fx=scale, fy=scale)

                if swapped:
                    status, col = "L/R look SWAPPED -> press w", (0, 0, 255)
                elif ready:
                    status, col = "READY - press s", (0, 255, 0)
                elif not enough:
                    status, col = f"need >= {MIN_CORNERS} common corners", (0, 0, 255)
                else:
                    status, col = "hold still...", (0, 200, 255)
                hud(pv, [(f"common {len(common)}  L {len(cur)}  R {0 if ir is None else len(ir)}"
                          f"  saved {len(saved)}", (255, 255, 255)),
                         (status, col)], keys_help)
                k = pv.shape[1] / 1536.0
                for t, x in (("LEFT", int(10 * k)), ("RIGHT", pv.shape[1] // 2 + int(10 * k))):
                    cv2.putText(pv, t, (x, int(100 * k)), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * k,
                                (255, 255, 0), max(1, round(2 * k)), cv2.LINE_AA)

            cv2.imshow(WIN, pv)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            elif key == ord('s') and mode == "capture":
                if not ready:
                    why = ("左右眼反了，先按 w" if swapped else
                           "共同角点不够" if not enough else "板子还在动，停稳再按")
                    print("没保存：" + why)
                    continue
                nums = [int(n[:-4]) for n in saved if n[:-4].isdigit()]
                name = f"{max(nums, default=0) + 1:03d}.png"
                cv2.imwrite(os.path.join(ldir, name), L)
                cv2.imwrite(os.path.join(rdir, name), R)
                saved.append(name)
                coverage[name] = cl[np.isin(il, common)].tolist()
                print(f"保存 {name}  共同角点 {len(common)}  （共 {len(saved)} 对）")

            elif key == ord('d') and mode == "capture":
                if not saved:
                    print("没有可删除的图")
                    continue
                name = saved.pop()
                for d_ in (ldir, rdir):
                    p = os.path.join(d_, name)
                    if os.path.exists(p):
                        os.remove(p)
                coverage.pop(name, None)
                print(f"已删除 {name}（剩 {len(saved)} 对）")

            elif key == ord('w'):
                args.swap = not args.swap
                prev = None
                print(f"左右已交换，现在 swap={int(args.swap)}"
                      + (f"。注意：已有 {len(saved)} 对是按旧设置存的，建议删掉 {args.dir} 重拍"
                         if saved else ""))

            elif key == ord('c'):
                print(f"\n开始标定（{len(saved)} 对图，可能要几十秒，画面会暂停）...")
                if run_calib(args.square, args.dir, args.swap, args.flip, args.calib, args.k3):
                    maps = make_maps(load_calib(args.calib))
                    mode = "rect"
                    print("已切到校正画面，按 v 回到采集")

            elif key == ord('v'):
                if maps is None:
                    print("还没有标定结果，先拍图再按 c")
                else:
                    mode = "capture" if mode == "rect" else "rect"
                    prev = None
    finally:
        cap.release()
        cv2.destroyAllWindows()


def cmd_calib(args):
    if not run_calib(args.square, args.dir, args.swap, args.flip, args.calib, args.k3):
        sys.exit(1)


def cmd_view(args):
    c = load_calib(args.calib)
    check_setting(c, args)
    maps = make_maps(c)
    scale = preview_scale(args.preview)
    cap = open_cam(args.cam, args.exposure)
    setup_window(scale)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            L, R = split(frame, args.swap, args.flip)
            cv2.imshow(WIN, hud(rectified_preview(L, R, maps, scale), [], "q quit"))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


# ---------------------------------------------------------------- main
def common_args(p):
    p.add_argument("--cam", type=int, default=1, help="摄像头索引")
    p.add_argument("--square", type=float, default=28.0, help="打印后实测的方格边长 mm")
    p.add_argument("--dir", default="calib_imgs", help="标定图片保存目录")
    p.add_argument("--calib", default="stereo_calib.yaml", help="标定结果文件")
    p.add_argument("--swap", action="store_true",
                   help="交换左右半边。这台相机已标定确认左半边就是左眼，默认不交换")
    p.add_argument("--no-swap", dest="swap", action="store_false", help=argparse.SUPPRESS)  # 兼容旧命令
    p.add_argument("--flip", action="store_true", help="相机倒装：整帧先旋转 180°")
    p.add_argument("--preview", type=float, default=0, help="预览缩放，0=按屏幕自动")
    p.add_argument("--k3", action="store_true", help="同时估计 k3（默认固定为 0）")
    p.add_argument("--exposure", type=float, default=None, help="手动曝光，如 -8（1/256s）；默认自动")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="双目 ChArUco 标定")
    sub = ap.add_subparsers(dest="cmd")
    pb = sub.add_parser("board", help="生成 A4 标定板 PDF")
    pb.add_argument("--square", type=float, default=28.0)
    pb.add_argument("--out", default="charuco_A4.pdf")
    for name in ("live", "capture", "calib", "view"):
        common_args(sub.add_parser(name))

    argv = sys.argv[1:]
    if not argv or argv[0].startswith("-") and argv[0] not in ("-h", "--help"):
        argv = ["live"] + argv                 # 不写子命令时默认进入实时模式
    a = ap.parse_args(argv)
    {"board": cmd_board, "live": cmd_live, "capture": cmd_live,
     "calib": cmd_calib, "view": cmd_view}[a.cmd](a)
