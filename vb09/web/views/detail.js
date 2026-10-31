"use strict";
// vb09/web/views/detail.js —— 详情页 + 事件
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 详情页
// =====================================================================
// 详情页曲线标题 (穿环)
const LI = (c, txt, hh) => `<i style="background:${c}${hh ? ";height:8px" : ""}"></i>${txt}`;
const DET_LAB_GATE = [
  "距环平面 along（过环后为负）" + LI("var(--text)", "along") + LI("var(--tgt)", "环平面"),
  "高度（相对起飞点）" + LI("var(--text)", "飞机") + LI("var(--sp)", "设定点") + LI("var(--tgt)", "环开口", 1),
  "靶面偏差 m" + LI("var(--text)", "横 h") + LI("var(--sp)", "竖 v") + "<span class='dim'>（虚线 = 红点位置）</span>",
  "感知年龄 ms" + LI("var(--text)", "age_ms"),
  "环位置 σ cm" + LI("var(--ok)", "σ（各轴最大）"),
  "速度 m/s" + LI("var(--info)", "水平实测") + LI("var(--text)", "沿法向实测") + LI("var(--muted)", "沿法向指令") +
    LI("var(--bad)", "阶段硬限"),
];
let detLabSet = false;
function setDetLabels() {
  if (detLabSet) return;
  detLabSet = true;
  $$(".det-charts .lab").forEach((e, i) => { e.innerHTML = DET_LAB_GATE[i]; });
}
function renderDetailGate(t, d, W) {
  const G = gateNow(), g = G.geo;
  const vlim = t && t.envelope ? t.envelope[1] : null;
  drawChart($("#dc-d"), {series: [{data: H.ga, color: C.text}], lines: [{v: 0, color: C.tgt}], win: W, minSpan: 1});
  drawChart($("#dc-z"), {series: [{data: H.z, color: C.text}, {data: H.spz, color: C.sp}],
                         band: g && g.n ? [relv(g.c)[2] - g.r_in, relv(g.c)[2] + g.r_in] : null,
                         bandColor: C.band, win: W, minSpan: 0.5});
  drawChart($("#dc-ez"), {series: [{data: H.goh, color: C.text}, {data: H.gov, color: C.sp}],
                          lines: G.aimOff ? [{v: G.aimOff[0], color: C.text}, {v: G.aimOff[1], color: C.sp}] : [],
                          win: W, minSpan: 0.2});
  drawChart($("#dc-age"), {series: [{data: H.gage, color: C.text}], min: 0, win: W, minSpan: 100});
  drawChart($("#dc-g"), {series: [{data: H.gsd, color: C.ok}], min: 0, win: W, minSpan: 2});
  drawChart($("#dc-v"), {series: [{data: H.v, color: C.info}, {data: H.gva, color: C.text}, {data: H.gvc, color: C.muted}],
                         lines: [{v: vlim, color: C.bad}], win: W, minSpan: 0.5});
}
function renderDetail() {
  if (!S) return;
  loadColors();
  const t = tel(), p = P(), d = D() || {};
  const W = detWin;
  $("#det-info").textContent = t ? `阶段 ${PHASE[t.phase] || t.phase}` + (t.log_dir ? `  ·  日志 ${t.log_dir}` : "") : "飞行脚本未运行（只有 mavros 的曲线）";
  setDetLabels();
  renderDetailGate(t, d, W);
  renderRaw(t);
}
function fmtVal(v) {
  if (v === null || v === undefined) return "—";
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : (Math.abs(v) >= 100 ? v.toFixed(1) : v.toFixed(3));
  if (Array.isArray(v)) return v.map(fmtVal).join(", ");
  if (typeof v === "object") return Object.entries(v).map(([k, x]) => k + "=" + fmtVal(x)).join("  ");
  return String(v);
}
function kvTable(title, obj, skip) {
  if (!obj || !Object.keys(obj).length) return `<table class="kvt"><caption>${esc(title)}</caption><tr><td>—</td><td></td></tr></table>`;
  const rows = Object.entries(obj).filter(([k]) => !(skip || []).includes(k))
    .map(([k, v]) => `<tr><td>${esc(k)}</td><td>${esc(fmtVal(v))}</td></tr>`).join("");
  return `<table class="kvt"><caption>${esc(title)}</caption>${rows}</table>`;
}
function renderRaw(t) {
  const rates = {};
  for (const k of Object.keys(S.rates || {}))
    rates[k] = `${fx(S.rates[k], 1)} Hz` + (nz(S.ages[k]) ? `  (${fx(S.ages[k], 2)}s 前)` : "");
  const html = (t ? kvTable("飞行脚本", t, ["track", "safety", "gates", "gate", "t"]) +
                    (t.gate && Object.keys(t.gate).length ? kvTable("穿环 gate（控制）", t.gate, ["t"]) : "") +
                    kvTable("安全 safety", t.safety, ["t"]) : "")
    + (S.gate ? kvTable("/gate/status（感知）", S.gate, ["diag", "t"]) : "")
    + kvTable("话题频率", rates)
    + kvTable("MAVROS", Object.assign({}, S.mavros, {batt_v: S.batt_v, landed_state: S.landed}))
    + kvTable("位姿 (世界系)", S.pose, ["t"])
    + kvTable("速度", S.vel, ["t"]);
  $("#det-raw").innerHTML = html;
}

// ---------------- 事件 ----------------
function addEvent(e) {
  evLast = Math.max(evLast, e.id);
  const li = document.createElement("li");
  li.className = e.level;
  li.innerHTML = `<time>${hhmmss(e.t)}</time><span>${esc(e.msg)}</span>`;
  const ol = $("#ev-list");
  ol.prepend(li);
  while (ol.children.length > 300) ol.lastChild.remove();
}
