# -*- coding: utf-8 -*-
"""fig_single_target.py —— 把 14 项单目标测试结果做成一张验收图

左边表格（场景 / 到达交接 / 最大误差 / 耗时 / 判定），
右边放正常流程与遮挡流程的关键帧。
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "eval"))
from mcpv.cv import imwrite_unicode                        # noqa: E402
import bench_single_target as BST                          # noqa: E402

FONT = r"C:\Windows\Fonts\msyh.ttc"
OUT = ROOT / "eval" / "figures" / "single_target_report.jpg"


def main():
    cases = [
        ("A 正常流程 50→6.5 m", dict()),
        ("B 遮挡 0.3 s", dict(occl=(8.0, 8.3))),
        ("B 遮挡 0.5 s", dict(occl=(8.0, 8.5))),
        ("B 遮挡 1.0 s", dict(occl=(8.0, 9.0))),
        ("B 遮挡 2.0 s", dict(occl=(8.0, 10.0))),
        ("B 遮挡 3.0 s", dict(occl=(8.0, 11.0))),
        ("B 遮挡 5.0 s", dict(occl=(8.0, 13.0))),
        ("C 部分遮挡（半圆）", dict(partial=True)),
        ("D 光照变暗 50%", dict(darken=True)),
        ("E 旁边强反光", dict(glare=True)),
        ("F 画面抖动 2 px", dict(jitter=2.0)),
        ("G 相机平移 2 px/s", dict(camvx=2.0)),
        ("H 目标短暂出画 1 s", dict(out_of_frame=1.0)),
        ("I 接近速度突然加倍", dict(speedup_at=8.0)),
    ]
    rows = []
    print("跑 14 个场景…")
    for n, kw in cases:
        n = n.replace("正常流程 50→6.5 m", "正常流程 50→10 m")
        r = BST.run(n, **kw)
        rows.append(r)
        print(f"  {n:<22}{'是' if r['reached'] else '否':<4}"
              f"{r['worst']:5.0f}px {r['ms']:5.1f}ms {r['verdict']}")

    f_title = ImageFont.truetype(FONT, 26)
    f_head = ImageFont.truetype(FONT, 17)
    f_cell = ImageFont.truetype(FONT, 16)

    # 表格尺寸
    CW = [300, 110, 110, 100, 80]
    RH = 26
    TW = sum(CW)
    TH = RH * (len(rows) + 2) + 130

    tbl = Image.new("RGB", (TW, TH), (22, 22, 22))
    d = ImageDraw.Draw(tbl)
    d.text((8, 4), "单目标 14 场景实测（含 JPEG q85 + 噪声 σ3）",
           font=f_title, fill=(0, 220, 255))
    heads = ["场景", "到达交接", "最大误差", "耗时", "判定"]
    x = 0
    for i, h in enumerate(heads):
        d.rectangle([x, 36, x + CW[i] - 1, 36 + RH], fill=(40, 48, 58))
        d.text((x + 6, 40), h, font=f_head, fill=(200, 220, 240))
        x += CW[i]
    y = 36 + RH
    for r in rows:
        x = 0
        vals = [r["name"],
                "是" if r["reached"] else "否",
                f"{r['worst']:.0f} px",
                f"{r['ms']:.1f} ms",
                r["verdict"]]
        ok = r["verdict"] == "OK"
        for i, v in enumerate(vals):
            d.rectangle([x, y, x + CW[i] - 1, y + RH], outline=(60, 60, 60))
            col = (120, 230, 120) if ok else (240, 120, 120)
            d.text((x + 6, y + 4), v, font=f_cell,
                   fill=(230, 230, 230) if i == 0 else col)
            x += CW[i]
        y += RH
    n_ok = sum(1 for r in rows if r["verdict"] == "OK")
    d.text((8, y + 8), f"通过 {n_ok}/{len(rows)}   "
                       f"平均耗时 {np.mean([r['ms'] for r in rows]):.1f} ms/帧",
           font=f_head, fill=(0, 220, 255))
    d.text((8, y + 40), "红圆 20 cm（A4）：交接点 6.5 m → d_px 24.1；"
                        "硬底线 3 m → d_px 52.2", font=f_cell, fill=(200, 210, 220))
    d.text((8, y + 64), "红圆 26 cm（A3）：交接点 6.5 m → d_px 31.3；"
                        "硬底线 3 m → d_px 67.9", font=f_cell, fill=(200, 210, 220))
    d.text((8, y + 88), "换分辨率时 d_px 按 ×帧宽/1920 等比缩放；"
                        "改 --marker-cm 会自动重算", font=f_cell, fill=(160, 200, 230))

    # 右侧：两张演示拼版
    plates = []
    for name in ("occl_grid.jpg", "sim_grid.jpg"):
        p = ROOT / "eval" / "demo_out" / name
        if p.exists():
            im = cv2.imdecode(np.fromfile(str(p), dtype=np.uint8),
                              cv2.IMREAD_COLOR)
            if im is not None:
                plates.append(Image.fromarray(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)))

    W = TW + (plates[0].width if plates else 0) + 30
    H = max(TH, sum(p.height for p in plates) + 20) + 20
    sheet = Image.new("RGB", (W, H), (16, 16, 16))
    sheet.paste(tbl, (10, 10))
    yy = 10
    for p in plates:
        sheet.paste(p, (TW + 20, yy))
        yy += p.height + 6

    OUT.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(OUT, quality=92)
    print(f"出图 {OUT}  ({sheet.width}x{sheet.height})")


if __name__ == "__main__":
    main()
