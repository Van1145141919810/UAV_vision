# -*- coding: utf-8 -*-
"""bench_vlm_gate.py —— 选「VLM 语义闸门」用哪句问法

要解决的问题（2026-11-23 在 win06 上发现）
  红点检测器会把**阳台上的红花盆**当成目标，融合后仍然写"窗上有红点"。
  必须加一道语义闸门：裁红点周围一小块，让 MiniCPM-V 判断"这是不是贴在纸上的圆形标记"。

为什么不能直接问"是不是窗户"
  整帧问"能看到窗户吗" → 一律答"是"（负例 0/12）。
  必须**裁小块**；而且问的对象要是**那个红点**，不是窗户。

本脚本测 4 种问法，正例来自真实录像的红点，负例来自红点检测器在白天立面照上的误报。

用法
    python eval\\bench_vlm_gate.py
输出
    eval/vlm_gate.json + 一张对照图 eval/figures/vlm_gate.jpg
"""
from __future__ import annotations

import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "eval"))
sys.path.insert(0, str(Path(r"D:\Desktop\无人系统\_ref\s14")))

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
OUT_JSON = ROOT / "eval" / "vlm_gate.json"
OUT_FIG = ROOT / "eval" / "figures"

Q = {
    "Q1_有圆点吗":
        "画面正中心有东西吗？中心附近有没有一个【圆形的红点】？只回答一个字母：A=有，B=没有。",
    "Q2_纸上标记":
        "画面正中心有没有一个【贴在浅色（白/黄）纸板上的红色圆形标记】？只回答一个字母：A=有，B=没有。",
    "Q3_圆点还是杂物":
        "画面正中心那个红色东西是【一个人为画的圆形圆点】，还是【花、砖、衣服、招牌之类的杂物】？"
        "只回答一个字母：A=圆点，B=杂物。",
    "Q4_标记还是其他":
        "画面正中心那个红色东西，看起来像不像【用来做标记的红色圆点】（边缘整齐、接近正圆）？"
        "只回答一个字母：A=像标记，B=不像（是别的红色物体）。",
}


def ask(img, prompt, npd=4):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        return "", 0.0
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(URL, data=body, headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(rq, timeout=120) as r:
        txt = (json.loads(r.read()).get("response") or "").strip()
    return txt, time.time() - t0


def letter(txt, allowed="AB"):
    for ch in (txt or "").upper()[:8]:
        if ch in allowed:
            return ch
    return "?"


def crop_around(img, u, v, d_px, k=6.0, min_side=96, out=256):
    side = int(max(min_side, d_px * k))
    h, w = img.shape[:2]
    x0 = int(max(0, min(w - side, u - side // 2)))
    y0 = int(max(0, min(h - side, v - side // 2)))
    c = img[y0:y0 + side, x0:x0 + side]
    if c.size == 0:
        return None
    return cv2.resize(c, (out, out), interpolation=cv2.INTER_CUBIC)


def imwrite_u(path, img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if ok:
        Path(path).write_bytes(buf.tobytes())


def main():
    import bench_lat_crop as L
    from bench_window_range import scale_to_window, window_w
    from mcpv.cv import imread_unicode
    from mcpv.window_finder import _DotSource, FX, DOT_D_M

    rig_dir = Path(r"D:\Desktop\无人系统\_ref\s14")
    from vb09.perception.stereo_rig import StereoRig
    rig = StereoRig.from_file(str(rig_dir / "stereo_calib.yaml"), scale=1.0)
    ds = _DotSource(rig)
    assert ds.ok, "红点检测不可用"

    cases = []   # (标签, 真值, crop, 说明)
    # ── 正例：真实录像里的红点 ──────────────────────────────
    fr = L.load_frames()
    for target_d, lab in ((8.0, "40m"), (15.0, "22m"), (30.0, "11m")):
        k = min(fr, key=lambda q: abs(fr[q]["d"] - target_d))
        f = fr[k]
        Lr, _ = rig.rectify(f["clean"], f["clean"])
        cands = ds.find(Lr)
        if not cands:
            print("正例 %s 帧%d 检测不到，跳过" % (lab, k), flush=True)
            continue
        c = min(cands, key=lambda c: np.hypot(c["u"] - f["gx"], c["v"] - f["gy"]))
        crop = crop_around(Lr, c["u"], c["v"], max(8.0, c["d_px"]))
        if crop is not None:
            cases.append(("pos", "A", crop, "录像帧%d %s 点%.0fpx" % (k, lab, c["d_px"])))
    # ── 负例：白天立面照 + 负例图 上红点检测器的误报（花盆等）─────────
    NEGFILES = ["win01", "win02", "win03", "win04", "win05", "win06",
                "win07", "win08", "win09", "win10", "win11", "win12",
                "test1", "test2", "test3",
                "neg1", "neg2", "neg3", "neg4", "neg5"]
    for n in NEGFILES:
        p = ROOT / "data" / "samples" / (n + ".jpg")
        if not p.exists():
            continue
        im = imread_unicode(p)
        med, _ = window_w(im)
        if med is None:
            continue
        img = scale_to_window(im, med, 72)
        Lr, _ = rig.rectify(img, img)
        cands = ds.find(Lr)[:2]
        for i, c in enumerate(cands):
            crop = crop_around(Lr, c["u"], c["v"], max(10.0, c["d_px"]))
            if crop is not None:
                cases.append(("neg", "B", crop, "%s 误报#%d 点%.0fpx" % (n, i + 1, c["d_px"])))

    print("用例：正例 %d / 负例 %d\n" % (sum(1 for c in cases if c[0] == "pos"),
                                     sum(1 for c in cases if c[0] == "neg")), flush=True)

    rows = []
    t_all = time.time()
    for qname, qtext in Q.items():
        pos_ok = neg_ok = pos_n = neg_n = 0
        detail = []
        for tag, want, crop, desc in cases:
            ans, dt = ask(crop, qtext)
            got = letter(ans, "AB")
            ok = (got == want)
            detail.append(dict(desc=desc, want=want, got=got, ok=ok, raw=ans[:12]))
            if tag == "pos":
                pos_n += 1
                pos_ok += 1 if ok else 0
            else:
                neg_n += 1
                neg_ok += 1 if ok else 0
        rows.append(dict(q=qname, pos_ok=pos_ok, pos_n=pos_n, neg_ok=neg_ok, neg_n=neg_n,
                         detail=detail))
        print("%-14s 正例召回 %d/%d   负例排除 %d/%d   合计 %d/%d"
              % (qname, pos_ok, pos_n, neg_ok, neg_n, pos_ok + neg_ok, pos_n + neg_n), flush=True)

    json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    # 对照图：所有 crop 拼起来
    OUT_FIG.mkdir(parents=True, exist_ok=True)
    tiles = []
    for tag, want, crop, desc in cases:
        t = crop.copy()
        cv2.rectangle(t, (0, 0), (t.shape[1] - 1, 22), (0, 0, 0), -1)
        cv2.putText(t, ("%s %s" % (tag.upper(), want)), (4, 16), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0) if tag == "pos" else (0, 165, 255), 1)
        tiles.append(t)
    if tiles:
        while len(tiles) % 5:
            tiles.append(np.zeros_like(tiles[0]))
        rows_img = [np.hstack(tiles[i:i + 5]) for i in range(0, len(tiles), 5)]
        imwrite_u(OUT_FIG / "vlm_gate.jpg", np.vstack(rows_img))
        print("\n对照图 -> %s" % (OUT_FIG / "vlm_gate.jpg"))
    print("总耗时 %.0f s" % (time.time() - t_all))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
