"use strict";
// vb09/web/views/replay.js —— 回放
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 回放 (★2026-10-19): 读飞机上 ~/uav_logs/flight_*/ 的 csv (只读), 按时间拖动
//   上图固定看 flight_log.csv 的高度和水平速度; 下图任选一个 csv 的任意列 (按 t 列对齐)
//   冲刺只有一两秒, 实时页面看不清, 复盘主要靠这里
// =====================================================================
const REP = {flights: null, live: null, cur: null, main: null, spans: [], files: {}, file: null,
             cols: [], t0: 0, t1: 1, t: 0, playing: false, last: 0};
const REP_DEF_COLS = {"gate.csv": ["along", "off_h", "off_v"], "track.csv": ["d", "err_z"],
                      "flight_log.csv": ["vx", "vy"]};
const PHASE_COLOR = ph => isGatePhase(ph) || ph === "track" ? C.tgt : ph === "teleop" ? C.info :
  ph === "return_home" ? C.ok : ph === "takeoff" || ph === "hover" ? C.muted :
  ph === "descend" || ph === "auto_land" ? C.dim : C.text;
const logUrl = (d, f) => `/api/log?dir=${encodeURIComponent(d)}&file=${encodeURIComponent(f)}`;

async function initReplay(force) {
  if (REP.flights && !force) { renderReplay(); return; }
  try {
    const r = await getJSON("/api/logs");
    REP.flights = r.flights || [];
    REP.live = r.live;
    $("#rep-root").textContent = r.root;
    $("#rep-list").innerHTML = REP.flights.length ? REP.flights.map(f => {
      const kb = f.files.reduce((s, x) => s + x.size, 0) / 1024;
      const has = f.files.map(x => x.name).filter(n => /^(gate|track|flight_log)\.csv$/.test(n))
                         .map(n => n.replace(".csv", ""));
      const ts = f.name.replace(/^flight_(\d{4})(\d\d)(\d\d)_(\d\d)(\d\d)(\d\d).*$/, "$2-$3 $4:$5:$6");
      return `<div class="fl${REP.cur === f.name ? " on" : ""}" data-f="${esc(f.name)}"><b>${esc(ts)}</b>` +
             (f.name === r.live ? ' <span class="warn small">飞行中</span>' : "") +
             `<small>${esc(has.join(" · ") || "没有 csv")} · ${kb.toFixed(0)} KB</small></div>`;
    }).join("") : `<p class="muted small">没有飞行日志（${esc(r.root)}）</p>`;
  } catch (e) {
    $("#rep-list").innerHTML = '<p class="bad small">读不到日志列表（flight_monitor.py 是旧版？要和网页一起更新）</p>';
  }
  renderReplay();
}
function parseCSV(text) {
  const lines = text.split(/\r?\n/).filter(l => l.length);
  if (lines.length < 2) return null;
  const head = lines[0].split(",").map(s => s.trim());
  const cols = {};
  head.forEach(h => { cols[h] = []; });
  for (let i = 1; i < lines.length; i++) {
    const v = lines[i].split(",");
    if (v.length !== head.length) continue;              // 正在写的最后一行
    head.forEach((h, j) => {
      const x = parseFloat(v[j]);
      cols[h].push(Number.isFinite(x) ? x : (v[j] === "nan" || v[j] === "" ? null : v[j]));
    });
  }
  return {head, cols, n: cols[head[0]].length};
}
// 按时间二分找下标 (T 递增)
function idxAt(T, t) {
  let lo = 0, hi = T.length - 1;
  if (hi < 0) return -1;
  while (lo < hi) { const m = (lo + hi) >> 1; if (T[m] < t) lo = m + 1; else hi = m; }
  return lo > 0 && Math.abs(T[lo - 1] - t) < Math.abs(T[lo] - t) ? lo - 1 : lo;
}
async function loadFlight(name) {
  REP.cur = name; REP.files = {}; REP.main = null; REP.spans = []; REP.playing = false;
  $("#rep-play").textContent = "▶";
  for (const el of $$("#rep-list .fl")) el.classList.toggle("on", el.dataset.f === name);
  $("#rep-title").textContent = name;
  $("#rep-info").textContent = "加载中…";
  const f = (REP.flights || []).find(x => x.name === name);
  const csvs = f ? f.files.map(x => x.name).filter(n => n.endsWith(".csv")) : [];
  try {
    const r = await fetch(logUrl(name, "flight_log.csv"), {cache: "no-store"});
    if (r.ok) REP.main = parseCSV(await r.text());
  } catch (e) {}
  const M = REP.main;
  if (M && M.cols.t && M.n > 1) {
    const T = M.cols.t;
    REP.t0 = T[0]; REP.t1 = T[M.n - 1]; REP.t = REP.t0;
    // 阶段区间 (给曲线底色、轨迹着色)
    const ph = M.cols.phase || [];
    let s0 = 0;
    for (let i = 1; i <= M.n; i++)
      if (i === M.n || ph[i] !== ph[s0]) { REP.spans.push({a: T[s0], b: T[i - 1], ph: ph[s0]}); s0 = i; }
    const dur = REP.t1 - REP.t0;
    $("#rep-info").textContent = `${M.n} 行 · ${dur.toFixed(1)} s · 阶段 ` +
      [...new Set(ph)].map(p => PHASE[p] || p).join(" → ");
  } else $("#rep-info").textContent = "flight_log.csv 读不到或是空的";
  $("#rep-t").value = 0;
  const pref = ["gate.csv", "track.csv", "flight_log.csv"];
  const rank = n => (pref.indexOf(n) + 1) || 9;
  csvs.sort((a, b) => rank(a) - rank(b));
  $("#rep-file").innerHTML = csvs.map(n => `<option>${esc(n)}</option>`).join("");
  await loadRepFile(csvs[0]);
}
async function loadRepFile(fn) {
  REP.file = fn || null;
  if (fn && !(fn in REP.files)) {
    REP.files[fn] = null;
    try {
      const r = fn === "flight_log.csv" && REP.main ? null : await fetch(logUrl(REP.cur, fn), {cache: "no-store"});
      REP.files[fn] = r ? (r.ok ? parseCSV(await r.text()) : null) : REP.main;
    } catch (e) {}
  }
  const d = fn ? REP.files[fn] : null;
  const numeric = d ? d.head.filter(h => h !== "t" && h !== "t_wall" && d.cols[h].some(v => typeof v === "number")) : [];
  let saved = null;
  try { saved = JSON.parse(store("fm_rep_cols_" + fn) || "null"); } catch (e) {}
  REP.cols = (saved || REP_DEF_COLS[fn] || numeric.slice(0, 2)).filter(c => numeric.includes(c)).slice(0, 4);
  $("#rep-cols").innerHTML = !d ? `<span class="muted">${fn ? esc(fn) + " 读不到" : "没有 csv"}</span>` :
    numeric.map(h => `<label><input type="checkbox" value="${esc(h)}"${REP.cols.includes(h) ? " checked" : ""}> ${esc(h)}</label>`).join("");
  renderReplay();
}
const REP_COLORS = () => [C.text, C.sp, C.tgt, C.warn];
function renderReplay() {
  if (activeTab !== "rep") return;
  loadColors();
  const M = REP.main;
  drawRepTop($("#rp-top"), M);
  const xr = {x0: REP.t0, x1: REP.t1, cursor: REP.t, spans: REP.spans};
  if (M && M.cols.t) {
    const T = M.cols.t, pz = M.cols.pz || [], z0 = pz.find(v => typeof v === "number") || 0;
    const vx = M.cols.vx || [], vy = M.cols.vy || [];
    drawSeriesT($("#rp-c1"), Object.assign({series: [
      {x: T, y: pz.map(v => typeof v === "number" ? v - z0 : null), color: C.text},
      {x: T, y: vx.map((v, i) => typeof v === "number" && typeof vy[i] === "number" ? Math.hypot(v, vy[i]) : null), color: C.info},
    ]}, xr));
  } else drawSeriesT($("#rp-c1"), Object.assign({series: []}, xr));
  const d = REP.file ? REP.files[REP.file] : null;
  const cols = REP_COLORS();
  if (d && d.cols.t) drawSeriesT($("#rp-c2"), Object.assign({series: REP.cols.map((c, i) => ({x: d.cols.t, y: d.cols[c], color: cols[i]}))}, xr));
  else drawSeriesT($("#rp-c2"), Object.assign({series: [], note: d ? "这个文件没有 t 列，对不上时间" : ""}, xr));
  // 游标处的读数
  const parts = [`t+${(REP.t - REP.t0).toFixed(2)} s`];
  if (M && M.cols.t) {
    const i = idxAt(M.cols.t, REP.t), g = k => (M.cols[k] || [])[i];
    const ph = g("phase");
    parts.push(PHASE[ph] || ph || "");
    if (typeof g("pz") === "number") parts.push(`高 ${(g("pz") - (M.cols.pz.find(v => typeof v === "number") || 0)).toFixed(2)} m`);
    if (typeof g("vx") === "number") parts.push(`水平 ${Math.hypot(g("vx"), g("vy")).toFixed(2)} m/s`);
  }
  if (d && d.cols.t) {
    const i = idxAt(d.cols.t, REP.t);
    REP.cols.forEach((c, k) => {
      const v = d.cols[c][i];
      parts.push(`<span style="color:${cols[k]}">${esc(c)} ${typeof v === "number" ? v.toFixed(3) : esc(v ?? "—")}</span>`);
    });
  }
  $("#rep-now").innerHTML = parts.join(" · ");
}
function drawSeriesT(cv, o) {
  const {ctx, w, h} = prep(cv);
  if (w < 20 || h < 20) return;
  const pad = {l: 46, r: 8, t: 6, b: 16};
  const x0 = o.x0, x1 = o.x1 > o.x0 ? o.x1 : o.x0 + 1;
  const X = t => pad.l + (t - x0) / (x1 - x0) * (w - pad.l - pad.r);
  ctx.font = "10px " + css("--mono");
  for (const sp of o.spans || []) {               // 底色: 跟踪 / 穿环段
    if (!(sp.ph === "track" || isGatePhase(sp.ph))) continue;
    ctx.fillStyle = C.band; ctx.fillRect(X(sp.a), pad.t, Math.max(1, X(sp.b) - X(sp.a)), h - pad.t - pad.b);
  }
  let lo = Infinity, hi = -Infinity;
  for (const s of o.series) for (const v of s.y || []) if (typeof v === "number") { if (v < lo) lo = v; if (v > hi) hi = v; }
  if (!Number.isFinite(lo)) { lo = 0; hi = 1; }
  if (hi - lo < 0.1) { const m = (hi + lo) / 2; lo = m - 0.05; hi = m + 0.05; }
  const sp = hi - lo; lo -= sp * 0.08; hi += sp * 0.08;
  const Y = v => pad.t + (1 - (v - lo) / (hi - lo)) * (h - pad.t - pad.b);
  ctx.strokeStyle = C.grid; ctx.fillStyle = C.dim; ctx.textAlign = "right"; ctx.lineWidth = 1;
  for (const v of [lo, (lo + hi) / 2, hi]) {
    ctx.beginPath(); ctx.moveTo(pad.l, Y(v)); ctx.lineTo(w - pad.r, Y(v)); ctx.stroke();
    ctx.fillText(v.toFixed(Math.abs(hi - lo) < 2 ? 2 : 1), pad.l - 4, Y(v) + 3);
  }
  if (lo < 0 && hi > 0) { ctx.strokeStyle = C.line2; ctx.beginPath(); ctx.moveTo(pad.l, Y(0)); ctx.lineTo(w - pad.r, Y(0)); ctx.stroke(); }
  ctx.textAlign = "left"; ctx.fillText("0 s", pad.l, h - 3);
  ctx.textAlign = "right"; ctx.fillText(`${(x1 - x0).toFixed(1)} s`, w - pad.r, h - 3);
  for (const s of o.series) {
    const xs = s.x || [], ys = s.y || [], n = Math.min(xs.length, ys.length);
    const step = Math.max(1, Math.floor(n / (2 * w)));
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.4; ctx.beginPath();
    let pen = false;
    for (let i = 0; i < n; i += step) {
      const v = ys[i];
      if (typeof v !== "number" || typeof xs[i] !== "number") { pen = false; continue; }
      const px = X(xs[i]), py = Y(v);
      if (pen) ctx.lineTo(px, py); else { ctx.moveTo(px, py); pen = true; }
    }
    ctx.stroke();
  }
  if (nz(o.cursor)) {
    ctx.strokeStyle = C.warn; ctx.lineWidth = 1.2; ctx.beginPath();
    ctx.moveTo(X(o.cursor), pad.t); ctx.lineTo(X(o.cursor), h - pad.b); ctx.stroke();
  }
  if (o.note) { ctx.fillStyle = C.dim; ctx.textAlign = "center"; ctx.fillText(o.note, w / 2, h / 2); }
  if (!o.series.length && !o.note) { ctx.fillStyle = C.dim; ctx.textAlign = "center"; ctx.fillText("无数据", w / 2, h / 2); }
  ctx.textAlign = "left";
}
// 回放俯视图: 原点 = 日志第一行 (preflight 之后才开始记, 约等于起飞点); 按阶段着色
function drawRepTop(cv, M) {
  const {ctx, w, h} = prep(cv);
  if (w < 20 || h < 20) return;
  ctx.font = "11px " + css("--mono");
  if (!M || !M.cols.px || !M.cols.t) {
    ctx.fillStyle = C.dim; ctx.textAlign = "center";
    ctx.fillText(REP.cur ? "flight_log.csv 读不到" : "左边选一次飞行", w / 2, h / 2); ctx.textAlign = "left"; return;
  }
  const PX = M.cols.px, PY = M.cols.py, T = M.cols.t, PH = M.cols.phase || [];
  const i0 = PX.findIndex(v => typeof v === "number");
  if (i0 < 0) return;
  const ox = PX[i0], oy = PY[i0];
  let x0 = 0, x1 = 0, y0 = 0, y1 = 0;           // 轨迹包围盒 (相对原点), 画面中心放在包围盒中心
  for (let i = 0; i < M.n; i++) if (typeof PX[i] === "number" && typeof PY[i] === "number") {
    x0 = Math.min(x0, PX[i] - ox); x1 = Math.max(x1, PX[i] - ox);
    y0 = Math.min(y0, PY[i] - oy); y1 = Math.max(y1, PY[i] - oy);
  }
  const R = Math.max(1.0, (x1 - x0) / 2, (y1 - y0) / 2) * 1.15;
  const mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
  const s = Math.min(w, h - 16) / 2 / R, cx = w / 2, cy = (h - 16) / 2;
  const X = x => cx + (x - ox - mx) * s, Y = y => cy - (y - oy - my) * s;
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  const stepm = niceStep(R / 3), Ox = X(ox), Oy = Y(oy);
  for (let r = stepm; r < R * 3; r += stepm) { ctx.beginPath(); ctx.arc(Ox, Oy, r * s, 0, Math.PI * 2); ctx.stroke(); }
  ctx.beginPath(); ctx.moveTo(Ox, 0); ctx.lineTo(Ox, h - 16); ctx.moveTo(0, Oy); ctx.lineTo(w, Oy); ctx.stroke();
  const step = Math.max(1, Math.floor(M.n / 3000));
  ctx.lineWidth = 2;
  for (let i = i0 + step; i < M.n; i += step) {
    const j = i - step;
    if (typeof PX[i] !== "number" || typeof PX[j] !== "number") continue;
    ctx.strokeStyle = PHASE_COLOR(PH[i]); ctx.globalAlpha = T[i] <= REP.t ? 1 : 0.3;
    ctx.beginPath(); ctx.moveTo(X(PX[j]), Y(PY[j])); ctx.lineTo(X(PX[i]), Y(PY[i])); ctx.stroke();
  }
  ctx.globalAlpha = 1;
  const k = idxAt(T, REP.t);
  if (k >= 0 && typeof PX[k] === "number") {
    const yaw = typeof (M.cols.yaw || [])[k] === "number" ? M.cols.yaw[k] * Math.PI / 180 : 0;
    ctx.save(); ctx.translate(X(PX[k]), Y(PY[k])); ctx.rotate(-yaw);
    ctx.fillStyle = C.text; ctx.beginPath();
    ctx.moveTo(12, 0); ctx.lineTo(-7, 7); ctx.lineTo(-3, 0); ctx.lineTo(-7, -7); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  ctx.fillStyle = C.dim; ctx.fillText(`原点 = 日志第一行 · 网格 ${+stepm.toFixed(2)} m`, 4, h - 3);
  const legend = [["跟踪/穿环", C.tgt], ["手动", C.info], ["返航", C.ok], ["起飞/悬停", C.muted]];
  let lx = w - 4; ctx.textAlign = "right";
  for (const [txt, col] of legend.reverse()) { ctx.fillStyle = col; ctx.fillText(txt, lx, 12); lx -= ctx.measureText(txt).width + 10; }
  ctx.textAlign = "left";
}
$("#rep-list").addEventListener("click", e => { const d = e.target.closest(".fl"); if (d) loadFlight(d.dataset.f); });
$("#rep-refresh").addEventListener("click", () => initReplay(true));
$("#rep-file").addEventListener("change", e => loadRepFile(e.target.value));
$("#rep-cols").addEventListener("change", e => {
  let sel = $$("#rep-cols input:checked").map(x => x.value);
  if (sel.length > 4) { e.target.checked = false; sel = sel.filter(v => v !== e.target.value); }
  REP.cols = sel;
  store("fm_rep_cols_" + REP.file, JSON.stringify(sel));
  renderReplay();
});
$("#rep-t").addEventListener("input", e => { REP.t = REP.t0 + (REP.t1 - REP.t0) * e.target.value / 1000; renderReplay(); });
function repTick(now) {
  if (!REP.playing) return;
  REP.t = Math.min(REP.t1, REP.t + (now - REP.last) / 1000 * 2);
  REP.last = now;
  $("#rep-t").value = Math.round((REP.t - REP.t0) / Math.max(1e-6, REP.t1 - REP.t0) * 1000);
  renderReplay();
  if (REP.t >= REP.t1) { REP.playing = false; $("#rep-play").textContent = "▶"; }
  else requestAnimationFrame(repTick);
}
$("#rep-play").addEventListener("click", () => {
  if (!REP.main) return;
  REP.playing = !REP.playing;
  $("#rep-play").textContent = REP.playing ? "❚❚" : "▶";
  if (REP.playing) { if (REP.t >= REP.t1) REP.t = REP.t0; REP.last = performance.now(); requestAnimationFrame(repTick); }
});
