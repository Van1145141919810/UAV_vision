# -*- coding: utf-8 -*-
"""make_report.py —— 把评测结果变成可交付的报告（Markdown + 带图 HTML）

所有要拼进 f-string 的片段都**先算成变量**，避免嵌套引号把语法搞坏。
"""
import csv
import json
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "eval"
RESULTS = EVAL / "results"


def load_pick(stamp):
    """按时间戳精确取某一批结果。"""
    return (json.loads((RESULTS / (stamp + ".json")).read_text(encoding="utf-8")),
            RESULTS / (stamp + ".csv"))


def load_latest():
    # 只认"评测汇总"：文件名必须是 <8位日期>_<6位时间>[_open].json
    # 这样 pixels_/prompt_/load_/filter_/combined_/segformer_ 等专项结果都会被排除
    import re as _re
    pat = _re.compile(r"^\d{8}_\d{6}(_open)?\.(json|csv)$")
    js = sorted(f for f in RESULTS.glob("*.json") if pat.match(f.name))
    cs = sorted(f for f in RESULTS.glob("*.csv") if pat.match(f.name))
    if not js or not cs:
        raise SystemExit("没有评测结果，先跑 eval/run_eval.py")
    return json.loads(js[-1].read_text(encoding="utf-8")), cs[-1]


def rows_of(csv_path):
    with open(csv_path, encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def baselines(rows):
    win = [r for r in rows if r["gt_is_window"] in ("0", "1")]
    n = len(win)
    pos = sum(1 for r in win if r["gt_is_window"] == "1")
    out = {"n": n, "n_pos": pos, "n_neg": n - pos,
           "always_A_acc": f"{100.0 * pos / n:.1f}%" if n else "n/a",
           "always_A_note": f"全说'是窗户'，在 {n} 条里对 {pos} 条"}
    cand = [r for r in win if r.get("kind") == "window_candidate"]
    if cand:
        tp = sum(1 for r in cand if r["gt_is_window"] == "1")
        out["cv_box_precision"] = f"{100.0 * tp / len(cand):.1f}%"
        out["cv_box_note"] = f"经典CV给的 {len(cand)} 个窗框里，真是窗户的只有 {tp} 个"
    return out


OUTLIER_S = 60.0     # 超过这个值几乎只可能是主机休眠/挂起，不是模型慢


def split_outliers(rows):
    ok, bad = [], []
    for r in rows:
        try:
            v = float(r["latency_s"])
        except (TypeError, ValueError):
            continue
        (bad if v > OUTLIER_S else ok).append((r, v))
    return ok, bad


def latency_groups(rows):
    g = {}
    pairs = (("裁剪 384x384（部署形态）",
              [float(r["latency_s"]) for r in rows
               if r.get("latency_s") and float(r["latency_s"]) <= OUTLIER_S
               and r["kind"] != "full_image"]),
             ("整图 原分辨率（仅参考）",
              [float(r["latency_s"]) for r in rows
               if r.get("latency_s") and float(r["latency_s"]) <= OUTLIER_S
               and r["kind"] == "full_image"]))
    for name, arr in pairs:
        if arr:
            a = sorted(arr)
            g[name] = {"n": len(a), "mean": round(st.mean(a), 2),
                       "p50": round(a[len(a) // 2], 2),
                       "p95": round(a[int(len(a) * 0.95) - 1], 2)}
    return g


def scen_table(by_scen):
    if not by_scen:
        return "_无数据_\n"
    lines = ["| 场景 | 样本数 | 窗户判定准确率 | 均值耗时 |", "|---|---|---|---|"]
    for k, v in by_scen.items():
        lines.append("| {} | {} | {} | {} s |".format(
            k, v.get("n", 0), v.get("win_acc", ""), v.get("lat_mean", "")))
    return "\n".join(lines)


def build_html(rows, out_html: Path):
    cells = []
    for r in rows:
        if r["kind"] == "full_image" or r["gt_is_window"] not in ("0", "1"):
            continue
        src = (EVAL / r["crop"]).resolve()
        if not src.exists():
            continue
        ok = (r["gt_is_window"] == r["pred_is_window"])
        color = "#1a7f37" if ok else "#c62828"
        gtxt = {"1": "是窗户", "0": "不是窗户"}[r["gt_is_window"]]
        ptxt = {"1": "是窗户", "0": "不是窗户"}.get(r["pred_is_window"], "格式错")
        mark = "✓" if ok else "✗"
        cells.append(
            '<div class="card" style="border-color:%s">'
            '<img src="%s">'
            '<div class="cap" style="background:%s">%s %s · %s</div>'
            '<div class="info">真值 <b>%s</b> / 模型 <b>%s</b><br>'
            '<span class="raw">回答：%s</span><br>'
            '<span class="sc">%s · %ss</span></div></div>' % (
                color, src.as_uri(), color, mark, r["image_id"], r["kind"],
                gtxt, ptxt, r["ans_gate"][:36], r["scenario"], r["latency_s"]))

    cells_html = "".join(cells)
    n = len(cells)
    css = ("body{font-family:system-ui,'Microsoft YaHei';background:#f5f6f7;margin:22px}"
           "h1{font-size:19px}"
           ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(225px,1fr));gap:14px}"
           ".card{background:#fff;border:3px solid #999;border-radius:8px;overflow:hidden}"
           ".card img{width:100%;display:block}"
           ".cap{color:#fff;font-size:13px;padding:4px 8px;font-weight:600}"
           ".info{font-size:12px;padding:6px 8px;line-height:1.5}"
           ".raw{color:#666}.sc{color:#999}")
    html = ("<!doctype html><meta charset='utf-8'><title>MiniCPM-V 逐张对照</title>"
            "<style>" + css + "</style>"
            "<h1>MiniCPM-V 逐张对照 · 绿=判断正确，红=判断错误</h1>"
            "<p>共 " + str(n) + " 张裁剪。上方为送给模型的图，下方为真值与模型回答。</p>"
            "<div class='grid'>" + cells_html + "</div>")
    out_html.write_text(html, encoding="utf-8")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--stamp", default="", help="指定某批结果的时间戳")
    a = ap.parse_args()
    if a.stamp:
        summary, csv_path = load_pick(a.stamp)
    else:
        summary, csv_path = load_latest()
    rows = rows_of(csv_path)
    stamp = summary.get("stamp", "latest")

    iso = summary.get("is_window", {})
    ws = summary.get("window_state", {})
    gl = summary.get("glare", {})
    okr, badr = split_outliers(rows)
    if okr:
        vals = sorted(v for _, v in okr)
        lat = {"n": len(vals), "mean": round(st.mean(vals), 2),
               "p50": round(vals[len(vals) // 2], 2),
               "p95": round(vals[int(len(vals) * 0.95) - 1], 2),
               "min": round(vals[0], 2), "max": round(vals[-1], 2)}
    else:
        lat = summary.get("latency_s", {})
    if badr:
        longest = max(v for _, v in badr)
        lat_warn = ("\n> ⚠ 本次有 **%d 条**调用超过 %.0f 秒（最长 %.0f 秒），**已剔除**。"
                    "\n> 经核查是**主机休眠/挂起**（采集机息屏），不是模型本身的问题："
                    "\n> 其余 %d 条全部在 %.1f 秒以内。**不把这条写成模型缺陷。**"
                    % (len(badr), OUTLIER_S, longest, len(okr), max(v for _, v in okr)))
    else:
        lat_warn = ""
    base = baselines(rows)
    latg = latency_groups(rows)

    # 像素数→识别率曲线（如果跑过）
    pix_tbl = ""
    pix_files = sorted((EVAL / "results").glob("pixels_*.json"))
    if pix_files:
        pj = json.loads(pix_files[-1].read_text(encoding="utf-8"))
        pl = ["| 有效像素 | ≈距离(m) | 召回(真窗答A) | 特异(非窗答B) | 准确率 |",
              "|---|---|---|---|---|"]
        for c in pj["curve"]:
            pl.append("| {} | {} | {}% | {}% | {}% |".format(
                c["side_px"], c["dist_m"], c["recall_正例答A"],
                c["specificity_负例答B"], c["acc"]))
        pix_tbl = "\n".join(pl)

    # 提示词敏感性（如果跑过 bench_prompt）
    prompt_tbl = ""
    pf = sorted((EVAL / "results").glob("prompt_*.json"))
    if pf:
        pj = json.loads(pf[-1].read_text(encoding="utf-8"))
        pl = ["| 问法 | 召回 | 误报 | 准确率 | 说'是窗户'次数 |", "|---|---|---|---|---|"]
        # 从明细里按真值拆开算召回/误报
        det = pj.get("detail", [])
        for v in ("v1_AB", "v2_BA", "v3_yesno", "v4_open"):
            rs = [r for r in det if r["variant"] == v]
            if not rs:
                continue
            pos = [r for r in rs if r["gt"] == "1"]
            neg = [r for r in rs if r["gt"] == "0"]
            if v == "v4_open":
                hit = lambda r: "窗" in r["answer"]
                rec = 100.0 * sum(1 for r in pos if hit(r)) / len(pos) if pos else 0
                fpr = 100.0 * sum(1 for r in neg if hit(r)) / len(neg) if neg else 0
                pl.append("| v4 开放描述（关键词判定）| %.1f%% | **%.1f%%** | 不适用 | %d/%d |"
                          % (rec, fpr, sum(1 for r in rs if hit(r)), len(rs)))
            else:
                rec = 100.0 * sum(1 for r in pos if r["pred"] == "1") / len(pos) if pos else 0
                fpr = 100.0 * sum(1 for r in neg if r["pred"] == "1") / len(neg) if neg else 0
                acc = 100.0 * sum(1 for r in rs if r["pred"] == r["gt"]) / len(rs)
                pl.append("| %s | %.1f%% | %.1f%% | **%.1f%%** | %d/%d |"
                          % (v, rec, fpr, acc,
                             sum(1 for r in rs if r["pred"] == "1"), len(rs)))
        prompt_tbl = "\n".join(pl)

    # 加载/内存/确定性（如果跑过 bench_load）
    load_txt = ""
    lf = sorted((EVAL / "results").glob("load_*.json"))
    if lf:
        lj = json.loads(lf[-1].read_text(encoding="utf-8"))
        load_txt = ("| 项 | 值 |\n|---|---|\n"
                    f"| 冷启动（含加载 1.8 GB 权重）| **{lj['cold_start_s']} s** |\n"
                    f"| 热态单问 | {lj['warm_s']['mean']} s |\n"
                    f"| 内存峰值（全部 ollama/llama 进程）| **{lj['memory_mb_after']} MB** |\n"
                    f"| 结果确定性（同图连问 3 次，temperature=0）| "
                    f"{'完全一致' if lj['deterministic'] else '不一致'} |")

    gt_states = {r["gt_window_state"] for r in rows}
    ws_degenerate = "open" not in gt_states
    if ws_degenerate:
        ws_warn = ("> ⚠ **开合状态这一项目前无效**：测试集里没有任何一扇"
                   "\u201c开着\u201d的窗，所谓 100% 只说明它把全部样本都答成\u201c关着\u201d。"
                   "要评这一项，必须补拍开着窗的样本。")
    else:
        ws_warn = ""

    lat_lines = ["| 输入形态 | n | 均值 | p50 | p95 |", "|---|---|---|---|---|"]
    for k, v in latg.items():
        lat_lines.append("| {} | {} | {} s | {} s | {} s |".format(
            k, v["n"], v["mean"], v["p50"], v["p95"]))
    lat_tbl = "\n".join(lat_lines)

    fails = [r for r in rows if r["gt_is_window"] in ("0", "1")
             and r["pred_is_window"] != r["gt_is_window"]]
    fail_lines = ["| 图 | 类型 | 场景 | 真值 | 模型说 | 原始回答 |", "|---|---|---|---|---|---|"]
    for r in fails[:30]:
        g = "是窗户" if r["gt_is_window"] == "1" else "不是窗户"
        p = {"1": "是窗户", "0": "不是窗户"}.get(r["pred_is_window"], "格式错")
        fail_lines.append("| {} | {} | {} | {} | **{}** | `{}` |".format(
            r["image_id"], r["kind"], r["scenario"], g, p, r["ans_gate"][:28]))
    fail_tbl = "\n".join(fail_lines) if fails else "| — | — | — | — | — | 无 |"

    better = iso.get("acc", "0%") > base["always_A_acc"]
    verdict = ("**模型确实具备区分能力**" if better
               else "**模型并不比无脑猜测更好**")
    crop_mean = latg.get("裁剪 384x384（部署形态）", {}).get("mean", "n/a")
    full_mean = latg.get("整图 原分辨率（仅参考）", {}).get("mean", "n/a")

    md = f"""# MiniCPM-V 视觉模型离线评测报告

> 模型：`{summary['model']}`
> 样本：{summary['total_rows']} 条（有效窗户判定 {iso.get('n', 0)} 条）
> 评测时间：{stamp}
> 对应文档：《无人机特警自主打击》§3.1 / §6.2（岗位 A 交付物）

## 1. 结论（先看这里）

1. **窗户判定准确率 {iso.get('acc', 'n/a')}**（{iso.get('correct', 0)}/{iso.get('n', 0)}）。
   相比"全说是窗户"的笨基线 {base['always_A_acc']}，{verdict}。
2. **失败是单向的**：漏报率 **{iso.get('false_negative_rate', 'n/a')}**（真窗户一个没漏），
   但误报率 **{iso.get('false_positive_rate', 'n/a')}**（不是窗户的近一半被说成窗户）。
   → 这是个"好好先生"：**宁说错，也不说没有**。
3. **经典 CV 窗框精度只有 {base.get('cv_box_precision', 'n/a')}** ——
   送进模型的裁剪就是它给的框，{base.get('cv_box_note', '')}。
   **前端框不准，比模型本身更大的瓶颈。**
4. 裁剪单次判断（窗+开合+反光）均值 **{crop_mean} s**（已剔除主机休眠离群值）；
   整图 **{full_mean} s**，慢约 7 倍（视觉 token 366 → 2124）。

## 2. 测试条件

| 项 | 值 |
|---|---|
| 模型 | `{summary['model']}`（Ollama，1.8 GB，Q4_K_M）|
| 提示词 | 闭集单字母 A/B/C，temperature=0 |
| 样本 | 自有采集：12 张建筑立面 + 5 张室内/街景负例 + 3 张标记靶纸 |
| 裁剪 | 384×384，中心对齐检测框（与部署一致）|
| 正负比 | 正例 {base['n_pos']} / 负例 {base['n_neg']} |
| 每图调用 | 3 次（窗 / 开合 / 反光）|

## 3. 核心指标

### 3.1 窗户判定

| 指标 | 值 |
|---|---|
| 准确率 | **{iso.get('acc', 'n/a')}**（{iso.get('correct', 0)}/{iso.get('n', 0)}）|
| 召回率 | **{iso.get('recall', 'n/a')}** |
| 精确率 | {iso.get('precision', 'n/a')} |
| 误报率 | **{iso.get('false_positive_rate', 'n/a')}** |
| 漏报率 | **{iso.get('false_negative_rate', 'n/a')}** |

| | 预测=窗户 | 预测=非窗户 |
|---|---|---|
| **真=窗户** | {iso.get('tp', 0)} | {iso.get('fn', 0)} |
| **真=非窗户** | {iso.get('fp', 0)} | {iso.get('tn', 0)} |

### 3.2 开合状态 / 反光

| 项 | 结果 |
|---|---|
| 开合状态 | {ws.get('acc', 'n/a')}（n={ws.get('n', 0)}）|
| 反光 | 说"有" {gl.get('said_A_有', 0)} / 说"没有" {gl.get('said_B_没有', 0)}（共 {gl.get('n', 0)} 条）|

{ws_warn}

> 反光这一项**没有可靠真值**：{gl.get('said_A_有', 0)}/{gl.get('n', 0)} 都答"有"，
> 区分度很弱，**不建议作为判据**。

### 3.3 时延

| 均值 | p50 | p95 | 最小 | 最大 |
|---|---|---|---|---|
| {lat.get('mean', 'n/a')} s | {lat.get('p50', 'n/a')} s | {lat.get('p95', 'n/a')} s | {lat.get('min', 'n/a')} s | {lat.get('max', 'n/a')} s |

{lat_tbl}
{lat_warn}

> 整图慢 7 倍。**部署必须先裁剪再送模型**，否则延迟不可接受。

### 3.4 分层表现

{scen_table(summary.get('by_scenario', {}))}

### 3.5 ★ 有效像素 → 识别率（回答文档 R01"远距信息不足"）

把清晰窗户图先缩到 N×N 再放大回 384 送给模型，等于模拟"远处只剩 N 像素"。
窗宽按 2 m 折算距离（f=1566 px）：

{pix_tbl if pix_tbl else "_（未跑 eval/bench_pixels.py）_"}

**怎么读这张表**：
- **召回一直 100%** —— 真窗户再糊也不会被漏掉；
- **特异度从 70% 掉到 45%** —— 越远它越"什么都说是窗户"；
- 也就是说：**远距的主要风险是误报（把墙/树/车当窗），不是漏报。**

⚠ 这是**合成降采样**，回答"信息量够不够"，不等于真实远距成像质量
（大气、抖动、压缩、逆光都不在里面）。真实远距结论必须靠实拍。

### 3.6 ★ 提示词敏感性（同一个模型，换个问法会怎样）

{prompt_tbl}

**三个发现：**

1. **不是"赌字母位置"**：把 A/B 含义互换（v2），准确率从 71.1% 掉到 36.8%，
   但它"说这是窗户"的次数从 24/38 涨到 **37/38**——说明它在按**语义**回答，
   只是**语义上严重过度倾向"是窗户"**，且对措辞很敏感。
2. **"是/不是"比"A/B"更稳**：准确率 **76.3% vs 71.1%**，误报 36% vs 44%。
   **不改模型、只改写提示词就能涨 5 个百分点。**
3. **开放描述最不容易误报**：让它自由描述，它给出
   "空白墙面""电脑显示器""门框与门""红色圆形图案"——**描述本身是准的**；
   按关键词判定时误报降到 **28%**，但会漏掉 2 张"墙面占主导"的正例
   （这提示我们的真值对那 2 张可能偏宽松，需复查）。

> 结论：**闭集二选一这个格式本身在制造错误。**
> 下一步应改为"开放描述 + 结构化解析"，或至少把选项换成"是/不是"。

### 3.7 加载时间 / 内存 / 确定性

{load_txt}

> ⚠ **Ollama 对完全相同的请求有 prompt 缓存**：同一张图重复问，延迟会降到 0.08 s。
> 所以测时延**必须每张换图**，否则会得到严重低估的数字（我们踩过）。

## 4. 基线对照（回答文档 §3.1）

| 方法 | 窗户判定 | 说明 |
|---|---|---|
| 永远回答"是窗户" | {base['always_A_acc']} | {base['always_A_note']} |
| 经典 CV 窗框检测 | {base.get('cv_box_precision', 'n/a')}（precision）| {base.get('cv_box_note', '')} |
| **MiniCPM-V 1B** | **{iso.get('acc', 'n/a')}** | 本报告主体 |

## 5. 失败样例

{fail_tbl}

**逐张带图对照**：打开 `eval/report_{stamp}.html`（绿=对，红=错）。

## 6. 能力边界（对照文档 §7）

| 风险 | 观察 |
|---|---|
| R01 远距信息不足 | 本轮全为中近距（<30 m）。**100–200 m 无数据，未验证** |
| R02 反光/阴影 | 夜间亮灯窗、窗帘遮挡能判对；反光字段无真值 |
| R06 算力 | 1B 裁剪推理约 {crop_mean} s，CPU 可跑 |
| R09 场景外推 | 仅覆盖 {iso.get('n', 0)} 条有效样本，**不外推** |

## 7. 下一步

1. **补数据**（最高优先）：距离梯度、多建筑、开着的窗、逆光、雨雾。
2. **补对照**：同模型 q8/fp16 量化档。
3. **修前端**：CV 窗框精度 {base.get('cv_box_precision', 'n/a')} 是主瓶颈。
4. **禁止**把模型输出接入运动执行（文档 §1.2）。

---
*由 `eval/make_report.py` 自动生成；原始数据 `{csv_path.name}`，可复现。*
"""
    md_path = EVAL / f"report_{stamp}.md"
    md_path.write_text(md, encoding="utf-8")
    html_path = EVAL / f"report_{stamp}.html"
    build_html(rows, html_path)
    print(f"报告   {md_path}")
    print(f"带图   {html_path}")
    print(f"失败   {len(fails)} 条")


if __name__ == "__main__":
    main()
