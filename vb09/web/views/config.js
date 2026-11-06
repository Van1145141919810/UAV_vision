"use strict";
// vb09/web/views/config.js —— 起飞配置
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 起飞配置
// =====================================================================
let SCHEMA = null, baseKnobs = {}, baseName = "", defaults = null, lastPreviewOk = false;
let baseSets = "";        // 载入档案时的 set. 行 (判断"改过没有")

// ---- 配置页表单状态: 存浏览器, 刷新/重开页面还在; 同时决定监控页用的参考配置 (REF) ----
const normSets = t => String(t || "").split("\n").map(l => l.trim()).filter(Boolean).join("\n");
function sameKnobs(a, b) {
  a = a || {}; b = b || {};
  for (const k of new Set([...Object.keys(a), ...Object.keys(b)])) {
    const x = a[k] ?? "", y = b[k] ?? "";
    if (x === "" && y === "") continue;
    if (String(x) === String(y)) continue;
    if (x !== "" && y !== "" && +x === +y) continue;
    return false;
  }
  return true;
}
function formState() {
  return {base: baseName, baseKnobs, baseSets, knobs: readKnobs(), sets: $("#cfg-sets").value};
}
const dirtyOf = st => !sameKnobs(st.knobs, st.baseKnobs) || normSets(st.sets) !== normSets(st.baseSets);
const refLabel = st => st.base ? st.base + (dirtyOf(st) ? " *已改" : "")
                               : (dirtyOf(st) ? "未存的配置" : "代码默认值");
function loadFormState() {
  try { const v = JSON.parse(store("fm_cfg_form") || "null"); return v && typeof v === "object" ? v : null; }
  catch (e) { return null; }
}
function setRef(r, st) {
  // ★2026-10-19: 带上 task —— 脚本没在跑时监控页靠它选布局 (curTask)
  REF = {params: r.params, derived: r.derived, task: r.task || (r.derived && r.derived.task) || null,
         label: refLabel(st), dirty: dirtyOf(st), stale: false};
  requestRender();
}
// 页面打开时 (还没进配置页) 就把上次在配置页选的那套恢复成参考配置
async function restoreRef() {
  const st = loadFormState();
  if (!st) return;
  let knobs = st.knobs || {}, sets = st.sets || "";
  if (st.base && !dirtyOf(st)) {        // 没改过 → 以磁盘上的档案为准 (可能别人改过)
    try {
      const r = await getJSON("/api/site?name=" + encodeURIComponent(st.base));
      if (r.ok) { knobs = r.knobs || {}; sets = (r.sets || []).join("\n"); }
    } catch (e) {}
  }
  const r = await previewRaw({knobs, sets});
  if (!REF && r && r.ok && r.derived) setRef(r, st);   // 配置页已经先给了就不覆盖
}
let pvTimer = null, pvSeq = 0;
const DEFAULT_OF = {   // 旋钮 → 从默认 dry-run 结果里取"默认值"作占位
  z_low: r => r.derived && r.derived.z_band && r.derived.z_band[0],
  z_high: r => r.derived && r.derived.z_band && r.derived.z_band[1],
  takeoff_h: r => r.params.TAKEOFF_HEIGHT, xy_radius: r => r.params.GATE_XY_MAX,
  // ★2026-10-19 穿环 (stage12)
  dash_v: r => r.params.DASH_V, d_back: r => r.params.D_BACK, d_rear: r => r.params.D_REAR,
  d_side: r => r.params.D_SIDE, l_f: r => r.params.L_F, l_r: r => r.params.L_R,
  w_half: r => r.params.W_HALF, dash_acc: r => r.params.DASH_ACC, brake_acc: r => r.params.BRAKE_ACC,
  pass_run: r => r.params.PASS_RUN, back_margin: r => r.params.M_BACK, brake_over: r => r.params.BRAKE_OVER_T,
  t_lag: r => r.params.T_LAG, e_along: r => r.params.E_ALONG, v_tear_min: r => r.params.V_TEAR_MIN,
  entry_ang_max: r => r.params.ENTRY_ANG_MAX, d_straight: r => r.params.D_STRAIGHT,
  run_max: r => r.params.D_RUN_MAX,
};
// ---------------------------------------------------------------------------
// ★2026-11-06 表单按"目标 + 模式"组织 (模式元数据来自 /api/schema 的 modes = vb09/flight/mode_spec.py)
//   · 目标 / 模式: 顶部两组按钮 (背后仍是 data-k 的 <select>, readKnobs / 档案格式不变)
//   · 旋钮的 section (区) 决定归属: 只显示当前模式用到的区; 没有 section 的 (现场 / 高度 / 水平) 总显示
//   · 别的区里填了值的旋钮: 不显示, 值保留 (存档案照样写), 表单底部一行"另有 N 项"
//   · 这里不写任何模式名 / 区名: 加模式只改 mode_spec.py
// ---------------------------------------------------------------------------
const TASK_OF = {"穿环": "gate"};
function formTask() {
  const el = document.querySelector('[data-k="task"]');
  return (el && TASK_OF[el.value]) || (defaults && defaults.task) || "gate";
}
const MS = () => (SCHEMA && SCHEMA.modes) || {modes: [], sections: [], default_mode: "dash"};
function modeSpec(key) { return MS().modes.find(m => m.key === key) || null; }
function formMode() {
  // 和 profile/gate.knob_activity 同一规则: set.GATE_SHADOW = True (旧写法) 优先, 再看 mode 旋钮, 再看代码默认
  if (/^\s*GATE_SHADOW\s*=\s*(True|true|1)\s*$/m.test(($("#cfg-sets") || {}).value || "")) return "shadow";
  const el = document.querySelector('[data-k="mode"]');
  const m = el && el.value && MS().modes.find(x => x.label === el.value);
  return m ? m.key : ((defaults && defaults.params && defaults.params.MISSION_MODE) || MS().default_mode);
}
function formTarget() {
  const el = document.querySelector('[data-k="target"]');
  const t = el && el.value && (SCHEMA.targets || []).find(x => x.label === el.value);
  return t ? t.key : ((defaults && defaults.params && defaults.params.TARGET_KIND) || "ring");
}
const secLabel = s => ((MS().sections.find(x => x.key === s) || {}).label) || s;
function applyTaskFilter() {
  if (!SCHEMA) return;
  const task = formTask(), mode = formMode(), spec = modeSpec(mode);
  const act = new Set(spec ? spec.sections : MS().sections.map(s => s.key));
  const hiddenSet = [];
  for (const k of SCHEMA.knobs) {
    const f = document.querySelector(`.f[data-f="${k.key}"]`);
    if (!f) continue;
    const el = f.querySelector("[data-k]");
    const offTask = Array.isArray(k.tasks) && !k.tasks.includes(task);
    const offMode = !!k.section && !act.has(k.section);
    f.hidden = (offTask && !(el && el.value !== "")) || offMode;
    f.classList.toggle("other-task", offTask);
    if (offMode && el && el.value !== "") hiddenSet.push(k);
  }
  for (const fs of $$("#cfg-form fieldset")) fs.hidden = ![...fs.querySelectorAll(".f")].some(f => !f.hidden);
  const adv = $("#cfg-adv");
  if (adv) {
    const vis = [...adv.querySelectorAll(".f")].filter(f => !f.hidden);
    adv.hidden = !vis.length;
    const sm = adv.querySelector("summary");
    if (sm) sm.textContent = `高级：${[...new Set(vis.map(f => f.closest("fieldset").dataset.legend))].join(" / ")}`
                             + `（${vis.length} 项，空着 = 按上面的旋钮推导）`;
  }
  // 别的模式的设置: 只报数, 不显示 (值留在表单里, 存档案照样写进去)
  const hb = $("#cfg-hidden");
  if (hb) {
    hb.hidden = !hiddenSet.length;
    hb.textContent = hiddenSet.length ? `另有 ${hiddenSet.length} 项其他模式的设置（已保留在档案里，当前模式不用）：`
      + hiddenSet.map(k => k.label).join("、") : "";
    hb.title = hiddenSet.map(k => `${k.label} = ${document.querySelector(`[data-k="${k.key}"]`).value}  （${secLabel(k.section)}）`).join("\n");
  }
  renderPickers();
  const ta = $("#cfg-sets");
  if (ta) ta.placeholder = "GATE_HOLD_S=2";
}
// ---- 目标 / 模式 按钮组 ----
function renderPickers() {
  const box = $("#cfg-pick");
  if (!box || !SCHEMA) return;
  const mode = formMode(), tgt = formTarget(), spec = modeSpec(mode);
  const elM = document.querySelector('[data-k="mode"]'), elT = document.querySelector('[data-k="target"]');
  const btns = (el, items, cur, isDefault) => items.map(it =>
    `<button type="button" class="seg${it.key === cur ? " on" : ""}" data-pick="${esc(el.dataset.k)}" data-v="${esc(it.label)}"
      title="${esc(it.desc || "")}">${esc(it.label)}${it.key === cur && isDefault ? '<em>默认</em>' : ""}</button>`).join("");
  const sup = ((SCHEMA.target_support || {})[tgt] || {})[mode];
  const supTxt = !SCHEMA.target_support || !SCHEMA.target_support[tgt] ? ""
    : !sup ? `<span class="sw danger">这种目标不支持这个模式 —— 校验会拒绝</span>`
    : sup[0] === "experimental" ? `<span class="sw warnn">实验性${sup[1] ? "：" + esc(sup[1]) : ""}</span>` : "";
  const shadowOld = /^\s*GATE_SHADOW\s*=\s*(True|true|1)\s*$/m.test($("#cfg-sets").value || "");
  box.innerHTML = `
    <div class="pick-row"><span class="pick-l">目标</span><div class="segs">${btns(elT, SCHEMA.targets || [], tgt, !elT.value)}</div></div>
    <div class="pick-row"><span class="pick-l">模式</span><div class="segs">${btns(elM, MS().modes, mode, !elM.value && !shadowOld)}</div></div>
    <div class="pick-desc">${spec ? esc(spec.desc) : ""}
      ${spec ? `<span class="sw ${spec.drives ? "danger" : "okk"}">${spec.drives ? "这一趟会冲刺" : "不冲刺"}</span>` : ""}
      ${shadowOld ? `<span class="sw warnn">set.GATE_SHADOW = True（旧写法）→ 按影子飞</span>` : ""}
      ${supTxt}</div>`;
}
document.addEventListener("click", e => {
  const b = e.target.closest("button[data-pick]");
  if (!b) return;
  const el = document.querySelector(`[data-k="${b.dataset.pick}"]`);
  el.value = b.dataset.v;
  markChanged(); applyTaskFilter(); schedulePreview();
});

async function initConfig() {
  try { SCHEMA = await getJSON("/api/schema"); }
  catch (e) { $("#cfg-form").innerHTML = '<p class="bad">取不到旋钮表（flight_profile.py 不在？）</p>'; return; }
  MODE_INFO = SCHEMA.modes || MODE_INFO;
  buildForm();
  const st = loadFormState();
  if (st) {
    baseName = st.base || ""; baseKnobs = st.baseKnobs || {}; baseSets = st.baseSets || "";
  }
  await refreshSites();
  if (st) {
    const exists = baseName && [...$("#cfg-base").options].some(o => o.value === baseName);
    if (exists && !dirtyOf(st)) await loadSite();          // 没改过 → 重新读磁盘上的档案
    else {
      if (baseName && !exists) { baseName = ""; baseKnobs = {}; baseSets = ""; }
      setKnobs(st.knobs || {}); $("#cfg-sets").value = st.sets || "";
    }
    suggestName();
  }
  // 先跑一次"全空"= 代码默认值, 用来填占位
  const r = await previewRaw({knobs: {}, sets: ""});
  if (r && r.params) {
    defaults = r;
    for (const k of SCHEMA.knobs) {
      const el = document.querySelector(`[data-k="${k.key}"]`);
      if (!el || !DEFAULT_OF[k.key]) continue;
      const v = DEFAULT_OF[k.key](r);
      if (nz(v)) el.placeholder = "默认 " + (+v.toFixed(3));
    }
  }
  applyTaskFilter();
  schedulePreview(0);
}
const PICKERS = new Set(["mode", "target"]);       // 用按钮组显示的 choice 旋钮
function buildForm() {
  // 分组: 有 section 的按区 (legend = 区名), 没有的按旋钮表的 group。adv:true 的进「高级」折叠区
  const gkey = k => k.section ? "§" + k.section : k.group;
  const glabel = g => g.startsWith("§") ? secLabel(g.slice(1)) : g;
  const fs = ks => {
    const groups = [];
    for (const k of ks) { if (!groups.includes(gkey(k))) groups.push(gkey(k)); }
    return groups.map(g => `<fieldset data-legend="${esc(glabel(g))}"><legend>${esc(glabel(g))}</legend>${
      ks.filter(k => gkey(k) === g).map(field).join("")}</fieldset>`).join("");
  };
  const ks = SCHEMA.knobs.filter(k => !PICKERS.has(k.key));
  const basic = ks.filter(k => !k.adv), adv = ks.filter(k => k.adv);
  const hiddenSel = SCHEMA.knobs.filter(k => PICKERS.has(k.key)).map(k =>
    `<select data-k="${k.key}" hidden><option value=""></option>${k.choices.map(c => `<option>${esc(c)}</option>`).join("")}</select>`).join("");
  const form = $("#cfg-form");
  form.innerHTML = `<div id="cfg-pick" class="pick"></div>${hiddenSel}` + fs(basic) +
    (adv.length ? `<details id="cfg-adv"><summary>高级</summary>${fs(adv)}</details>` : "") +
    `<div id="cfg-hidden" class="note small" hidden></div>`;
  form.addEventListener("input", () => { markChanged(); schedulePreview(); });
  form.addEventListener("change", () => { markChanged(); applyTaskFilter(); schedulePreview(); });
  $("#cfg-sets").addEventListener("input", () => applyTaskFilter());
  applyTaskFilter();
}
function field(k) {
  let input;
  if (k.type === "choice")
    input = `<select data-k="${k.key}"><option value="">（默认）</option>${k.choices.map(c => `<option>${esc(c)}</option>`).join("")}</select>`;
  else if (k.type === "float")
    input = `<input type="number" step="any" data-k="${k.key}" ${nz(k.min) ? `min="${k.min}"` : ""} ${nz(k.max) ? `max="${k.max}"` : ""}>`;
  else input = `<input type="text" data-k="${k.key}" maxlength="40">`;
  const range = [nz(k.min) ? "≥" + k.min : "", nz(k.max) ? "≤" + k.max : ""].filter(Boolean).join(" ");
  return `<label class="f" data-f="${k.key}"><span class="fl">${esc(k.label)}${k.unit ? `<em>${esc(k.unit)}</em>` : ""}</span>${input}
    <small>${esc(k.help)}${range ? ` <span class="dim">(${range})</span>` : ""}</small></label>`;
}
function readKnobs() {
  const out = {};
  for (const el of $$("#cfg-form [data-k]")) if (el.value !== "") out[el.dataset.k] = el.value;
  return out;
}
function setKnobs(kn) {
  for (const el of $$("#cfg-form [data-k]")) el.value = kn[el.dataset.k] ?? "";   // 包括隐藏的 (别的模式的值保留)
  // 档案里填了高级项 → 自动展开, 免得"看不见但生效"
  const adv = $("#cfg-adv");
  if (adv && SCHEMA) adv.open = adv.open || SCHEMA.knobs.some(k => k.adv && kn[k.key] !== undefined && kn[k.key] !== "");
  markChanged();
  applyTaskFilter();
}
function markChanged() {
  for (const el of $$("#cfg-form [data-k]")) {
    const b = baseKnobs[el.dataset.k];
    const cur = el.value;
    const same = (b === undefined || b === null) ? cur === "" : String(b) === cur || (+b === +cur && cur !== "");
    const f = el.closest(".f");
    if (f) f.classList.toggle("chg", !same);
    else {                                        // 目标 / 模式按钮组: 和档案不同就给按钮组描黄边
      const b = document.querySelector(`button[data-pick="${el.dataset.k}"]`);
      if (b) b.parentElement.classList.toggle("chg", !same);
    }
  }
}
async function refreshSites() {
  try {
    const r = await getJSON("/api/sites");
    const bad = r.bad || [];
    $("#cfg-dir").textContent = r.dir + (bad.length ? `  ⚠ 跳过 ${bad.length} 个读不了的: ${bad.map(b => b.name).join(", ")}` : "");
    $("#cfg-dir").title = bad.map(b => `${b.name}.conf  ${b.error}`).join("\n");
    $("#cfg-dir").classList.toggle("warn", bad.length > 0);
    $("#cfg-base").innerHTML = `<option value="">（不载入，从代码默认值开始）</option>` +
      r.sites.map(s => `<option value="${esc(s.name)}">${esc(s.name)}  · ${new Date(s.mtime * 1000).toLocaleString()}</option>`).join("");
    if (baseName) $("#cfg-base").value = baseName;
  } catch (e) {}
}
async function loadSite() {
  const name = $("#cfg-base").value;
  if (!name) { baseKnobs = {}; baseName = ""; baseSets = ""; $("#cfg-sets").value = ""; setKnobs({}); suggestName(); schedulePreview(0); return; }
  let r;
  try { r = await getJSON("/api/site?name=" + encodeURIComponent(name)); }
  catch (e) { r = {ok: false, error: "连不上飞机上的监控服务"}; }
  // ★2026-10-21 读不出来就停在这里, 不要把表单清空成代码默认值还显示"校验通过"
  if (!r || !r.knobs) { alert("档案「" + name + "」载入失败：" + ((r && r.error) || "未知错误") + "\n表单没有改动。"); return; }
  if (r.errors && r.errors.length) alert("档案有错误：\n" + r.errors.join("\n"));
  baseKnobs = r.knobs || {}; baseName = name;
  $("#cfg-sets").value = baseSets = (r.sets || []).join("\n");      // 档案里的 set.KEY 行
  setKnobs(baseKnobs); suggestName(); schedulePreview(0);
}
function suggestName() {
  const d = new Date(), p = n => String(n).padStart(2, "0");
  $("#sv-name").value = (baseName || "site") + `_${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`;
}
let pvKey = null;          // 上一次排进去的预览内容
function schedulePreview(ms) {
  // 改完一个框直接点按钮时, 框的 change 事件在 mousedown 那一刻触发; 如果这里再禁用一次按钮,
  // 这一下点击就被吞了。内容和刚排过的一样 (input 事件已经排过) → 不重跑。
  const key = JSON.stringify([readKnobs(), normSets($("#cfg-sets").value)]);
  if (ms === undefined && key === pvKey) return;
  pvKey = key;
  clearTimeout(pvTimer);
  lastPreviewOk = false; setSaveEnabled(!(true));
  $("#pv-verdict").className = "verdict busy"; $("#pv-verdict").textContent = "校验中…";
  pvTimer = setTimeout(runPreview, ms ?? 450);
}
async function previewRaw(body) {
  try {
    return await getJSON("/api/preview", {method: "POST", headers: {"Content-Type": "application/json"},
                                         body: JSON.stringify(body)});
  } catch (e) { return {ok: false, errors: ["连不上飞机上的监控服务"]}; }
}
async function runPreview() {
  const seq = ++pvSeq;
  const r = await previewRaw({knobs: readKnobs(), sets: $("#cfg-sets").value});
  if (seq !== pvSeq) return;            // 只显示最后一次
  const st = formState();
  try { store("fm_cfg_form", JSON.stringify(st)); } catch (e) {}
  if (r && r.ok && r.derived) setRef(r, st);
  else if (REF) REF.stale = true;      // 监控页继续画上一套通过的, 卡片上注明
  renderPreview(r);
}
let LAST_PV = null;       // 最近一次预览结果, 窗口尺寸变化时重画用
function renderPreview(r) {
  loadColors();
  LAST_PV = r;
  lastPreviewOk = !!r.ok;
  const armed = !$("#cfg-armed").hidden;
  setSaveEnabled(!(!r.ok || armed));
  $("#pv-time").textContent = new Date().toLocaleTimeString();
  const v = $("#pv-verdict");
  v.className = "verdict " + (r.ok ? "ok" : "bad");
  v.textContent = r.ok ? "✓ 校验通过 —— 这套配置可以起飞" : "✗ 拒绝起飞 —— 下面的问题必须先解决";
  $("#pv-err").innerHTML = (r.errors || []).map(e => `<li>${esc(e)}</li>`).join("");
  $("#pv-warn").innerHTML = (r.warnings || []).map(e => `<li>${esc(e)}</li>`).join("");
  // 标出出错的旋钮: 旋钮级错误带键名, 生效值级错误带中文名 (如"停靠距离")
  for (const f of $$(".f")) f.classList.remove("err");
  for (const e of (r.errors || [])) for (const k of (SCHEMA ? SCHEMA.knobs : []))
    if (e.startsWith(k.key + "=") || e.startsWith(k.key + " ") || e.includes(k.key + "(")
        || (k.label.length >= 3 && e.includes(k.label))) {
      const f = document.querySelector(`.f[data-f="${k.key}"]`); if (f) f.classList.add("err");
    }
  const d = r.derived, p = r.params || {};
  const isGate = (r.task || (d && d.task)) === "gate";
  // 开关 (2026-11-06: 按模式 —— 不冲刺的模式不列冲刺开关, 也不说"锁定后会冲刺")
  const sw = d && d.switches ? d.switches : null;
  const mi = (d && d.mode) || null;
  const dashSec = !mi || (mi.sections || []).includes("dash");
  $("#pv-sw").innerHTML = !sw ? "" : !isGate ? "" : !dashSec ? [
    `<span class="sw okk">${esc(mi.label)}：不冲刺</span>`,
    mi.support && mi.support[0] === "experimental" ? `<span class="sw warnn">目标 + 模式：实验性</span>` : "",
    `<span class="sw ${sw.GATE_ALLOW_SIM_DRIVE ? "danger" : "okk"}">${sw.GATE_ALLOW_SIM_DRIVE ? "SIM 识别可驱动！" : "SIM 识别拦截"}</span>`,
    `<span class="sw">${sw.GATE_REQUIRE_NORMAL ? "必须有法向 n" : "无法向按视线进入"}</span>`].join("") : [
    `<span class="sw ${sw.GATE_SHADOW ? "okk" : "danger"}">${sw.GATE_SHADOW ? "影子模式（不摆位不冲刺）" : "实飞：锁定后会冲刺"}</span>`,
    `<span class="sw ${sw.GATE_SLOW_POLICY === 2 ? "danger" : "okk"}">${
      {0: "场地不够：不降速（回手动段）", 1: "场地不够：降速要按 Y 确认", 2: "场地不够：自动降速"}[sw.GATE_SLOW_POLICY ?? 1]}</span>`,
    nz(sw.GATE_VIRTUAL_OFFSET) && sw.GATE_VIRTUAL_OFFSET > 0
      ? `<span class="sw info">虚拟环：前移 ${fx(sw.GATE_VIRTUAL_OFFSET, 1)} m，停在纸前</span>` : "",
    `<span class="sw ${sw.GATE_CONFIRM_DASH ? "okk" : "danger"}">${sw.GATE_CONFIRM_DASH ? "冲刺前按 G" : "锁定后直接冲刺"}</span>`,
    `<span class="sw ${sw.GATE_ALLOW_SIM_DRIVE ? "danger" : "okk"}">${sw.GATE_ALLOW_SIM_DRIVE ? "SIM 识别可冲刺！" : "SIM 识别拦截"}</span>`,
    `<span class="sw">${sw.GATE_REQUIRE_NORMAL ? "必须有法向 n" : "无法向按视线进入"}</span>`].join("");
  if (d) {
    const pvw = PREVIEWS[(mi && mi.preview) || "dash"] || PREVIEWS.dash;
    drawTop($("#pv-top"), {d, drone: {x: 0, y: 0, yaw: Math.PI / 2}, trail: [], mode: "fence", noLabel: true,
                           lim: d.phase_envelope && d.phase_envelope.takeoff ? d.phase_envelope.takeoff[0] : null,
                           marks: pvw.marks ? pvw.marks(d) : null});
    drawHeight($("#pv-h"), {d, z: d.takeoff_height});
    const pe = d.phase_envelope || {};
    const t = d.timeouts || {}, s = d.speeds || {}, sf = d.safety || {};
    const row = (k, val) => `<dt>${k}</dt><dd class="num">${val}</dd>`;
    $("#pv-env").innerHTML = pvw.rows(d, row);
  } else { $("#pv-env").innerHTML = ""; prep($("#pv-top")); prep($("#pv-h")); }
  $("#pv-applied").innerHTML = (r.applied || []).map(([k, v]) => `<div>${esc(k)} = ${esc(v)}</div>`).join("") || '<span class="muted">（无，全是代码默认值）</span>';
  $("#pv-lines").innerHTML = (r.lines || []).map(l => `<li>${esc(l)}</li>`).join("");
  if (!r.ok && r.stderr) $("#pv-err").innerHTML += `<li><pre class="small">${esc(r.stderr)}</pre></li>`;
}
// ★2026-11-06 右侧核对表按模式的 preview 选 (mode_spec.MODE_SPECS[..].preview)。加一种预览 = 这里加一项
const PREVIEWS = {
  dash: {rows: (d, row) => gateEnvRows(d, row)},
  dock: {rows: (d, row) => dockEnvRows(d, row), marks: d => dockMarks(d)},
};
function dockEnvRows(d, row) {
  const k = d.dock || {}, pe = d.phase_envelope || {}, sf = d.safety || {}, t = d.timeouts || {};
  const fitTxt = nz(k.d) && nz(d.xy_soft)
    ? (k.d <= d.xy_soft + 1e-9 ? `目标在起飞点前 ≤ ${fx(k.d + d.xy_soft)} m 都能停到（停靠点要在水平范围内）`
                               : "") : "";
  return [
    row("瞄点高度带", `${fx(d.z_band && d.z_band[0])} ~ ${fx(d.z_band && d.z_band[1])} m · 硬顶 ${fx(d.max_height, 1)} m`),
    row("起飞高度", `${fx(d.takeoff_height)} m`),
    row("水平", `范围 ${fx(d.xy_soft)} · 硬围栏 ${fx(d.xy_hard)} · 起降 ${fx(d.base_drift)} m`),
    row("停靠", `目标前 ${fx(k.d)} m${k.d_auto ? "（自动）" : ""} · 悬停 ${fx(k.hold, 0)} s`),
    fitTxt ? row("停靠点", esc(fitTxt)) : "",
    k.z_scan ? row("上下扫描", `高度 ${fx(k.z_scan[0])} → ${fx(k.z_scan[1])} m · 视角跨度 ` +
        (k.span_deg < k.span_min ? `<b class="warn">${fx(k.span_deg, 1)}°</b>（要 ≥ ${fx(k.span_min, 0)}°）`
                                 : `${fx(k.span_deg, 1)}°（要 ≥ ${fx(k.span_min, 0)}°）`))
             : row("上下扫描", "不扫（没有法向时按视线停靠）"),
    row("速度硬限", `gate ${fx(pe.gate && pe.gate[1])} m/s（按摆位速度）`),
    row("时间", `起飞 ${fx(t.TAKEOFF_TIMEOUT, 0)} · 降落 ${fx(t.DESCEND_TIMEOUT, 0)} s`),
    row("安全", `倾角停桨 ${fx(sf.MAX_TILT_DEG, 0)}° · 视觉断供 ${fx(sf.VISION_LOST_LAND_S)} s`),
    `<dt></dt><dd class="small muted">俯视图：目标画在起飞点正前方停靠距离处（示意），此时停靠点就是起飞点</dd>`,
  ].join("");
}
function dockMarks(d) {
  const k = d.dock || {};
  if (!nz(k.d)) return null;
  return [{x: 0, y: k.d, label: "目标（示意）", color: "tgt"}, {x: 0, y: 0, label: "停靠点", color: "ok"}];
}
// ★2026-10-19 穿环 (stage12) 的包络核对表。字段见 flight_profile._derived_gate
function gateEnvRows(d, row) {
  const g = d.gate || {}, st = d.site || {}, pe = d.phase_envelope || {}, sf = d.safety || {};
  const t = d.timeouts || {};
  const ch = (Array.isArray(d.speed_chain) ? d.speed_chain : []).filter(x => Array.isArray(x) && nz(x[1]));
  const m = ch.length ? ch.reduce((a, b) => (b[1] < a[1] ? b : a), ch[0]) : null;
  return [
    row("瞄点高度带", `${fx(d.z_band && d.z_band[0])} ~ ${fx(d.z_band && d.z_band[1])} m · 硬顶 ${fx(d.max_height, 1)} m`),
    row("起飞高度", `${fx(d.takeoff_height)} m`),
    row("水平", `范围 ${fx(d.xy_soft)} · 硬围栏 ${fx(d.xy_hard)} · 起降 ${fx(d.base_drift)} m`),
    row("场地", `环后 ${fx(st.D_BACK)} · 起飞点后方 ${fx(st.D_REAR)} · 两侧 ${fx(st.D_SIDE)} m`),
    row("机体", `机头 ${fx(st.L_F)} · 机尾 ${fx(st.L_R)} · 半宽 ${fx(st.W_HALF)} m → 瞄点离环心 ≤ ${fx(g.r_safe)} m`),
    row("冲刺", (nz(g.v_plan) && g.v_plan < g.dash_v - 1e-9
        ? `<b class="warn">${fx(g.v_plan, 1)} m/s</b>（环后空间限速，设定 ${fx(g.dash_v, 1)}）`
        : `${fx(g.dash_v, 1)} m/s`) +
        ` · 加 ${fx(g.dash_acc, 1)} / 减 ${fx(g.brake_acc, 1)} m/s² · 用时 ${fx(g.t_dash)} s`),
    nz(g.pass_run) ? row("过纸冲刺", `机头碰纸后再冲 ${fx(g.pass_run)} m 才刹车（+滞后补偿 ${fx(g.pass_lag)} m）`) : "",
    row("助跑 / 刹车", `起跑点距纸 ${fx(g.d_align)} m · 刹车起点 s=${fx(g.s_brake)} · 停止 s=${fx(g.overshoot)} m`),
    nz(g.back_margin) ? row("环后墙", (nz(g.brake_over) ? `含刹车超调 ${fx(g.brake_over)} m 后` : "") +
        `机头离墙 ${fx(g.back_margin + g.tol)} m（要求 ≥ ${fx(g.back_margin)}）` +
        (nz(g.d_back_need) ? ` · 按 ${fx(g.dash_v, 1)} m/s 冲要环后空间 ≥ ${fx(g.d_back_need)} m` : ""))
      : row("刹停窗口", Array.isArray(g.stop_window) ? `${fx(g.stop_window[0])}~${fx(g.stop_window[1])}，±${fx(g.tol)}` : "—"),
    row("撞纸速度", `${fx(g.v_hit)} m/s`),
    row("速度硬限", `gate ${fx(pe.gate && pe.gate[1])} m/s（冲刺前自动收紧）`),
    ch.length ? row("速度上限链", ch.map(x => `${esc(x[0])} ${fx(x[1])}`).join(" · ") +
                    (m ? ` → <b class="warn">实际 ≤ ${fx(m[1])} m/s（瓶颈：${esc(m[0])}）</b>` : "")) : "",
    row("时间", `起飞 ${fx(t.TAKEOFF_TIMEOUT, 0)} · 降落 ${fx(t.DESCEND_TIMEOUT, 0)} s`),
    row("安全", `倾角停桨 ${fx(sf.MAX_TILT_DEG, 0)}° · 视觉断供 ${fx(sf.VISION_LOST_LAND_S)} s`),
    `<dt></dt><dd class="small muted">以上按"正对进入"估算；起飞后按环的实际位置重新规划，场地放不下会自动降速</dd>`,
  ].join("");
}
function setSaveEnabled(en) { $("#sv-btn").disabled = $("#sv-go").disabled = $("#sv-temp").disabled = !en; }
function dangerOk(sets, verb) {
  const danger = [];
  if (/GATE_SHADOW\s*=\s*(False|false|0)\b/.test(sets)) danger.push("关闭穿环影子模式（锁定后会冲刺）");
  if (/GATE_ALLOW_SIM_DRIVE\s*=\s*(True|true|1)\b/.test(sets)) danger.push("允许 SIM 识别结果驱动冲刺");
  if (/GATE_CONFIRM_DASH\s*=\s*(False|false|0)\b/.test(sets)) danger.push("冲刺前不再等 G 确认");
  return !danger.length || confirm("这次配置包含危险开关：\n\n· " + danger.join("\n· ") + "\n\n确定" + verb + "？");
}
// 只生成启动脚本, 不存档案 (配置进飞机上的临时文件, 每次覆盖)
async function genTemp() {
  const sets = $("#cfg-sets").value;
  if (!dangerOk(sets, "生成")) return;
  setSaveEnabled(false);
  const msg = $("#sv-msg");
  msg.className = "small muted"; msg.textContent = "生成中（飞机上会再跑一次完整校验）…";
  const st = formState();
  let r;
  try {
    r = await getJSON("/api/launcher_temp", {method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({knobs: st.knobs, sets, base: st.base ? refLabel(st) : ""})});
  } catch (e) { r = {ok: false, error: "连不上飞机（flight_monitor.py 是旧版？要和这个网页一起更新）"}; }
  setSaveEnabled(lastPreviewOk && $("#cfg-armed").hidden);
  if (!r.ok) {
    msg.className = "small bad";
    msg.textContent = "✗ " + (r.error || "生成失败") + ((r.errors && r.errors.length) ? "：" + r.errors.join("；") : "");
    $("#sv-cmds").innerHTML = ""; return;
  }
  msg.className = "small ok";
  msg.textContent = `✓ 已生成 ${r.launcher.command}（临时配置「${refLabel(st)}」，没有存为档案）`;
  $("#sv-cmds").innerHTML = cmdRow("起飞：笔记本新开普通 SSH 窗口（tmux / VSCode 终端会被拒绝），登录飞机后直接运行",
                                   r.launcher.command);
  pollLauncher(true).then(renderLauncher);
}
async function save(withLauncher) {
  const name = $("#sv-name").value.trim();
  const sets = $("#cfg-sets").value;
  if (!dangerOk(sets, "保存")) return;
  setSaveEnabled(!(true));
  const msg = $("#sv-msg");
  msg.className = "small muted"; msg.textContent = "保存中（飞机上会再跑一次完整校验）…";
  let r;
  try {
    r = await getJSON("/api/save", {method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({name, knobs: readKnobs(), sets, overwrite: $("#sv-over").checked,
                            launcher: !!withLauncher})});
  } catch (e) { r = {ok: false, error: "连不上飞机"}; }
  setSaveEnabled(!(!lastPreviewOk));
  if (!r.ok) {
    msg.className = "small bad";
    msg.textContent = "✗ " + (r.error || "保存失败") + ((r.errors && r.errors.length) ? "：" + r.errors.join("；") : "");
    $("#sv-cmds").innerHTML = ""; return;
  }
  msg.className = "small ok";
  msg.textContent = "✓ 已保存 " + r.path + (r.launcher ? "，并生成启动脚本 " + r.launcher.command : "");
  $("#sv-cmds").innerHTML = r.launcher
    ? cmdRow("起飞：笔记本新开普通 SSH 窗口（tmux / VSCode 终端会被拒绝），登录飞机后直接运行", r.launcher.command)
    : cmdRow("1. 先核对（哪里都能跑）", r.dry_run_command) +
      cmdRow("2. 起飞 —— 必须在笔记本新开的普通 SSH 窗口里（tmux / VSCode 终端会被拒绝）", r.command);
  if (r.launcher) pollLauncher(true);
  baseName = name; baseKnobs = readKnobs(); baseSets = sets; markChanged(); refreshSites();
  const st = formState();
  try { store("fm_cfg_form", JSON.stringify(st)); } catch (e) {}
  if (REF) { REF.label = refLabel(st); REF.dirty = false; }
  HELP = null;                       // 帮助页的档案下拉框下次重新拉
}
// ---------------- 启动脚本状态 (和 fly.sh 运行时的自查用同一套判断) ----------------
let LN = null, lnFetchMs = 0;
async function pollLauncher(once) {
  if (activeTab === "cfg" || once) {
    try { LN = await getJSON("/api/launcher"); lnFetchMs = Date.now(); } catch (e) {}
  }
  if (!once) setTimeout(pollLauncher, 3000);
}
function renderLauncher() {
  const box = $("#ln-box");
  if (!LN) { box.textContent = ""; return; }
  if (!LN.exists) {
    box.className = "ln";
    box.innerHTML = `<span class="muted">还没有启动脚本 ${esc(LN.command)} —— 点「生成启动脚本」</span>`;
    return;
  }
  const age = (LN.now - LN.gen_ts) + (Date.now() - lnFetchMs) / 1000;
  const fa = s => s >= 3600 ? `${Math.floor(s / 3600)} 小时 ${Math.floor(s % 3600 / 60)} 分`
                : s >= 60 ? `${Math.floor(s / 60)} 分 ${Math.floor(s % 60)} 秒` : `${Math.max(0, Math.floor(s))} 秒`;
  const bad = [], warn = [];
  const what = LN.temp ? "临时配置" : "档案";
  if (LN.conf_missing) bad.push(what + "已不存在 —— 运行会被拒绝");
  else if (LN.conf_changed) warn.push(what + "在生成之后被改过" + (LN.temp ? "（别的电脑又生成过一次？）" : ""));
  if (LN.script_missing) bad.push("飞行脚本已不存在 —— 运行会被拒绝");
  else if (LN.script_changed) warn.push("飞行脚本在生成之后被改过");
  if (age > LN.max_age) warn.push(`已超过 ${LN.max_age} 秒`);
  const cls = bad.length ? "bad" : warn.length ? "warn" : "ok";
  const st = bad.length ? "✗ " + bad.join("；")
           : warn.length ? "⚠️ " + warn.join("；") + " —— 运行时会先警告，要输入 yes 才起飞"
           : `✓ 新鲜 —— ${fa(LN.max_age - age)} 内运行不会提示`;
  box.className = "ln " + cls;
  box.innerHTML = `<div>当前 <code>${esc(LN.command)}</code> → ${LN.temp ? "<b>临时配置</b>（没有存为档案）"
      : "档案 <b>" + esc(LN.site) + "</b>"}，${fa(age)}前生成
    （${new Date(LN.gen_ts * 1000).toLocaleTimeString()}）</div>
    <div><b class="st ${cls}">${esc(st)}</b></div>
    <div class="row"><button class="btn sm" id="ln-regen">${LN.temp ? "按这份临时配置重新生成" : "按这个档案重新生成"}</button>
      <button class="btn sm" data-copy="${esc(LN.command)}">复制命令</button></div>`;
  $("#ln-regen").onclick = regenLauncher;
}
async function regenLauncher() {
  if (!LN || !LN.site) return;
  const b = $("#ln-regen"); b.disabled = true; b.textContent = "校验中…";
  let r;
  try {
    r = await getJSON("/api/launcher", {method: "POST", headers: {"Content-Type": "application/json"},
                                        body: JSON.stringify({name: LN.site, temp: !!LN.temp})});
  } catch (e) { r = {ok: false, error: "连不上飞机"}; }
  if (!r.ok) alert("没有生成：" + (r.error || "") + ((r.errors && r.errors.length) ? "\n" + r.errors.join("\n") : ""));
  await pollLauncher(true);
  renderLauncher();
}
setInterval(() => { if (activeTab === "cfg") renderLauncher(); }, 1000);

function cmdRow(label, cmd) {
  return `<div class="small muted" style="margin-top:8px">${esc(label)}</div>
    <div class="cmd"><code>${esc(cmd)}</code><button class="btn" data-copy="${esc(cmd)}">复制</button></div>`;
}
document.addEventListener("click", async e => {
  const b = e.target.closest("[data-copy]"); if (!b) return;
  const txt = b.dataset.copy;
  try { await navigator.clipboard.writeText(txt); }
  catch (err) { const ta = document.createElement("textarea"); ta.value = txt; document.body.appendChild(ta); ta.select(); document.execCommand("copy"); ta.remove(); }
  const old = b.textContent; b.textContent = "已复制"; setTimeout(() => b.textContent = old, 1200);
});
