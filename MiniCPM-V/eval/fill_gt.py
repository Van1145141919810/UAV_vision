# -*- coding: utf-8 -*-
"""填 GT（按 eval/sheets/sheet1.jpg / sheet2.jpg 逐格人工判定）。

判定口径：**画面正中心主要是不是窗户**。中心是墙面/树/车/桌面的，一律 0，
即使画面角落有窗 —— 因为模型看到的就是那一小块的中心。
"""
import csv
from pathlib import Path

L = Path(r"D:\Desktop\无人系统\MiniCPM-V\eval\labels.csv")

# crop_stem: (is_window, window_state, glare, scenario, note)
GT = {
    "neg1_bg1": ("0", "n/a", "unknown", "indoor", "白墙"),
    "neg2_w1":  ("0", "n/a", "unknown", "indoor", "桌面显示器"),
    "neg2_w2":  ("0", "n/a", "unknown", "indoor", "键盘特写"),
    "neg3_w1":  ("1", "closed", "no", "outdoor_night", "夜间楼角，右侧有窗"),
    "neg3_w2":  ("0", "n/a", "unknown", "outdoor_night", "树叶"),
    "neg4_w1":  ("0", "n/a", "unknown", "indoor", "瓷砖墙+金属架"),
    "neg4_w2":  ("0", "n/a", "unknown", "indoor", "瓷砖墙"),
    "neg5_bg1": ("0", "n/a", "unknown", "indoor", "灰墙"),
    "test1_w1": ("0", "n/a", "unknown", "outdoor", "水池石头"),
    "test1_w2": ("0", "n/a", "unknown", "outdoor", "绿叶"),
    "test2_w1": ("0", "n/a", "unknown", "indoor", "办公室内景（门/人/打印机）"),
    "test2_w2": ("0", "n/a", "unknown", "indoor", "桌面显示器"),
    "test3_w1": ("0", "n/a", "unknown", "indoor", "白板+红色磁钉"),
    "test3_w2": ("0", "n/a", "unknown", "indoor", "红圆靶纸（我们的标记，不是窗）"),
    "win01_w1": ("0", "n/a", "unknown", "outdoor", "树木/汽车（误检框）"),
    "win01_w2": ("0", "n/a", "unknown", "outdoor", "树木/汽车（误检框）"),
    "win02_w1": ("1", "closed", "no", "outdoor", "立面多窗"),
    "win02_w2": ("0", "n/a", "unknown", "outdoor", "中心是墙，窗户在边缘"),
    "win03_w1": ("1", "closed", "no", "outdoor", "立面多窗"),
    "win03_w2": ("0", "n/a", "unknown", "outdoor", "中心是墙"),
    "win04_w1": ("1", "closed", "no", "outdoor", "立面多窗"),
    "win04_w2": ("1", "closed", "no", "outdoor", "横向窗带过中心"),
    "win05_w1": ("1", "closed", "no", "outdoor", "窗帘+空调"),
    "win05_w2": ("1", "closed", "no", "outdoor", "窗+格栅+空调"),
    "win06_w1": ("0", "n/a", "unknown", "outdoor", "墙面+树"),
    "win06_w2": ("1", "closed", "no", "outdoor", "立面窗"),
    "win07_w1": ("1", "closed", "no", "outdoor", "立面窗+绿植"),
    "win07_w2": ("1", "closed", "no", "outdoor", "立面窗+绿植"),
    "win08_w1": ("1", "closed", "no", "outdoor", "大窗台+花盆"),
    "win08_w2": ("1", "closed", "no", "outdoor", "大窗+窗帘"),
    "win09_w1": ("1", "closed", "no", "outdoor_night", "夜间亮灯窗"),
    "win09_w2": ("0", "n/a", "unknown", "outdoor_night", "电动车群（误检框）"),
    "win10_w1": ("0", "n/a", "unknown", "outdoor", "树+墙"),
    "win10_w2": ("0", "n/a", "unknown", "outdoor", "混凝土墙"),
    "win11_w1": ("0", "n/a", "unknown", "outdoor", "中心是墙"),
    "win11_w2": ("0", "n/a", "unknown", "outdoor", "树+墙"),
    "win12_w1": ("0", "n/a", "unknown", "outdoor", "墙面+栏杆"),
    "win12_w2": ("0", "n/a", "unknown", "outdoor", "墙面"),
}


def main():
    with open(L, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
        fields = list(rows[0].keys())

    filled = 0
    for r in rows:
        stem = Path(r["crop"]).stem
        if r["kind"] == "full_image":
            r["scenario"] = {}
            # 整图不填 is_window（中心真值没有意义），只用于时延与定性
            r["scenario"] = "full_image"
            r["note"] = "整图：只统计时延/稳定性，不参与窗户判定准确率"
            continue
        g = GT.get(stem)
        if not g:
            print(f"  ⚠ 没写 GT: {stem}")
            continue
        r["gt_is_window"], r["gt_window_state"], r["gt_glare"] = g[0], g[1], g[2]
        r["scenario"] = g[3]
        r["note"] = g[4]
        filled += 1

    with open(L, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"已填 {filled} 条裁剪的 GT → {L}")

    pos = sum(1 for r in rows if r["gt_is_window"] == "1")
    neg = sum(1 for r in rows if r["gt_is_window"] == "0")
    print(f"正例(是窗户) {pos} / 负例(不是窗户) {neg}")


if __name__ == "__main__":
    main()
