# -*- coding: utf-8 -*-
"""label_windows.py —— 窗框标注工具（OpenCV）

═══════════════════════════════════════════════════════════════════
怎么用（傻瓜版）
───────────────────────────────────────────────────────────────────
启动后会按顺序显示图片。

  拖鼠标左键    画一个新框（把一扇窗整个框进去）
  右键          删掉鼠标下那个框
  a             载入"机器猜的框"（青色），一次性加进你的标注
  z             撤销上一个框
  c             清空本图所有框
  s             保存本图
  n / 空格      下一张（自动保存）
  p             上一张
  q / Esc       退出并保存

  画完所有图后，标注存在 eval/labels_boxes/<图名>.json

═══════════════════════════════════════════════════════════════════
什么算"一扇窗"（口径，务必统一）
───────────────────────────────────────────────────────────────────
  ✅ 把玻璃区域**整个**框进去，包括窗框金属边
  ✅ 同一面墙上的每一扇窗都单独框一个
  ❌ 不要把空调外机框进去（它在窗户下方，是独立设备）
  ❌ 不要把整层楼的窗连成一个大框
  ❌ 墙面、门、楼梯、栏杆都不框
  ⚠ 被树叶挡掉一部分的窗，**仍然框**（后面训练时要学会处理遮挡）

═══════════════════════════════════════════════════════════════════
机器预猜（可选）
───────────────────────────────────────────────────────────────────
先跑一次 pred_boxes.py 生成预测，工具里按 a 就能一键载入，
然后你只需要"删错的、补漏的"——比从零画快很多。

用法：
  python eval/label_windows.py                      # 默认目录
  python eval/label_windows.py --dir D:\\Desktop\\window
"""
import argparse
import json
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402

OUT = ROOT / "eval" / "labels_boxes"
MAXW, MAXH = 1500, 950


def fit(img):
    h, w = img.shape[:2]
    s = min(MAXW / w, MAXH / h, 1.0)
    return cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA), s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="")
    ap.add_argument("--pred", default="", help="预猜框目录，默认 eval/labels_boxes/_pred")
    a = ap.parse_args()

    dirs = [Path(a.dir)] if a.dir else [ROOT / "data" / "samples",
                                       Path(r"D:\Desktop\window")]
    files = []
    for d in dirs:
        if d.exists():
            files += sorted(f for f in d.iterdir()
                            if f.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not files:
        raise SystemExit("没找到图片")

    OUT.mkdir(parents=True, exist_ok=True)
    pred_dir = Path(a.pred) if a.pred else OUT / "_pred"

    idx = 0
    while 0 <= idx < len(files):
        src = files[idx]
        img0 = imread_unicode(src)
        if img0 is None:
            idx += 1
            continue
        H0, W0 = img0.shape[:2]
        img, s = fit(img0)

        # 读已有标注
        jp = OUT / f"{src.stem}.json"
        boxes = []
        if jp.exists():
            try:
                d = json.loads(jp.read_text(encoding="utf-8"))
                boxes = [tuple(b) for b in d.get("boxes", [])]
            except Exception:
                pass

        preds = []
        pp = pred_dir / f"{src.stem}.json"
        if pp.exists():
            try:
                preds = [tuple(b) for b in json.loads(
                    pp.read_text(encoding="utf-8")).get("boxes", [])]
            except Exception:
                pass

        state = {"drag": None, "cur": None, "done": False, "redraw": True}

        # ★ 坐标系：内部一律用**原图坐标**保存；屏幕坐标只在显示/鼠标事件里出现。
        #   以前混用导致两个 bug：画完框跑到左上角（被多乘了一次 s）、右键删不掉
        #   （拿屏幕坐标去比原图坐标）。
        def on_mouse(e, x, y, flags, param):
            # 屏幕 → 原图
            ix, iy = x / s, y / s

            if e == cv2.EVENT_LBUTTONDOWN:
                state["drag"] = (ix, iy)          # 存原图坐标
                state["cur"] = (x, y)             # 画的时候用屏幕坐标
            elif e == cv2.EVENT_MOUSEMOVE and state["drag"]:
                state["cur"] = (x, y)
                state["redraw"] = True
            elif e == cv2.EVENT_LBUTTONUP and state["drag"]:
                x0, y0 = state["drag"]            # 原图坐标
                if abs(ix - x0) * s > 6 and abs(iy - y0) * s > 6:
                    boxes.append((min(x0, ix), min(y0, iy),
                                  abs(ix - x0), abs(iy - y0)))
                state["drag"] = state["cur"] = None
                state["redraw"] = True
            elif e == cv2.EVENT_RBUTTONDOWN:
                best, bd = None, 1e9
                for i, (bx, by, bw, bh) in enumerate(boxes):
                    if bx <= ix <= bx + bw and by <= iy <= by + bh:
                        d2 = abs(ix - bx - bw / 2) + abs(iy - by - bh / 2)
                        if d2 < bd:
                            best, bd = i, d2
                if best is not None:
                    boxes.pop(best)
                    state["redraw"] = True

        cv2.namedWindow("label", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("label", on_mouse)

        while True:
            if state["redraw"]:
                view = img.copy()
                for (bx, by, bw, bh) in preds:
                    sx, sy = int(bx * s), int(by * s)
                    cv2.rectangle(view, (sx, sy),
                                  (sx + int(bw * s), sy + int(bh * s)),
                                  (255, 255, 0), 1)
                for i, (bx, by, bw, bh) in enumerate(boxes):
                    sx, sy = int(bx * s), int(by * s)
                    cv2.rectangle(view, (sx, sy),
                                  (sx + int(bw * s), sy + int(bh * s)),
                                  (0, 220, 0), 2)
                    cv2.putText(view, str(i + 1), (sx + 3, sy + 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
                if state["drag"] and state["cur"]:
                    dx, dy = state["drag"]
                    cv2.rectangle(view, (int(dx * s), int(dy * s)), state["cur"],
                                  (0, 255, 255), 1)
                bar = f"[{idx+1}/{len(files)}] {src.name}   框 {len(boxes)}"
                if preds:
                    bar += f"   (机器预猜 {len(preds)} 个，按 a 载入)"
                cv2.rectangle(view, (0, 0), (view.shape[1], 28), (30, 30, 30), -1)
                cv2.putText(view, bar, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)
                cv2.putText(view, "拖=画框  右键=删  a=载入预猜  z=撤销  c=清空  "
                                  "s=存  n=下一张  p=上一张  q=退出",
                            (8, view.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (200, 200, 200), 2)
                cv2.imshow("label", view)
                state["redraw"] = False

            k = cv2.waitKey(20) & 0xFF
            if k in (ord("q"), 27):
                state["done"] = True
                break
            elif k == ord("a"):
                boxes += preds
                state["redraw"] = True
            elif k == ord("z") and boxes:
                boxes.pop()
                state["redraw"] = True
            elif k == ord("c"):
                boxes = []
                state["redraw"] = True
            elif k in (ord("s"), ord("n"), 32):
                json.dump({"image": str(src), "size": [W0, H0],
                           "boxes": [list(map(int, b)) for b in boxes]},
                          open(jp, "w", encoding="utf-8"), ensure_ascii=False)
                print(f"  保存 {src.name}：{len(boxes)} 个框")
                if k == ord("s"):
                    state["redraw"] = True
                else:
                    idx += 1
                    break
            elif k == ord("p"):
                json.dump({"image": str(src), "size": [W0, H0],
                           "boxes": [list(map(int, b)) for b in boxes]},
                          open(jp, "w", encoding="utf-8"), ensure_ascii=False)
                idx -= 1
                break

        if state["done"]:
            json.dump({"image": str(src), "size": [W0, H0],
                       "boxes": [list(map(int, b)) for b in boxes]},
                      open(jp, "w", encoding="utf-8"), ensure_ascii=False)
            break
        cv2.destroyWindow("label")

    cv2.destroyAllWindows()
    # 汇总
    total = 0
    for f in sorted(OUT.glob("*.json")):
        try:
            n = len(json.loads(f.read_text(encoding="utf-8")).get("boxes", []))
        except Exception:
            n = 0
        total += n
    print(f"\n标注目录 {OUT}")
    print(f"共 {len(list(OUT.glob('*.json')))} 张图，{total} 个框")


if __name__ == "__main__":
    main()
