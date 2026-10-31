"use strict";
// vb09/web/core.js —— 工具/全局状态
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 工具
// =====================================================================
const $ = s => document.querySelector(s);
const $$ = s => Array.from(document.querySelectorAll(s));
const nz = v => v !== null && v !== undefined && !Number.isNaN(v);
const fx = (v, d = 2, u = "") => nz(v) ? Number(v).toFixed(d) + u : "—";
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const esc = s => String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const hhmmss = t => { const d = new Date(t * 1000); return d.toTimeString().slice(0, 8); };
function store(k, v) { try { if (v === undefined) return localStorage.getItem(k); localStorage.setItem(k, v); } catch (e) { return null; } }

const PHASE = {idle:"空闲", init:"初始化", preflight:"起飞前检查", preflight_failed:"检查未通过",
  prime:"预热设定点", offboard_switch:"切 OFFBOARD", arm:"解锁", takeoff:"起飞", hover:"悬停",
  teleop:"手动", return_home:"返航", descend:"下降", auto_land:"自动降落",
  cleanup:"结束", handover:"飞手接管"};
const PHASE_HINT = {
  init: "飞行脚本初始化…",
  preflight: "起飞前检查中（MAVROS / 位姿 / VINS / 视觉 / 静止漂移 / COM_OBL_ACT）…",
  preflight_failed: "起飞前检查未通过 —— 原因在飞行终端里",
  prime: "预热设定点，马上切 OFFBOARD 并解锁", offboard_switch: "切 OFFBOARD…", arm: "解锁…",
  takeoff: "起飞中  ·  L = 就地降落  X = 停桨",
  hover: "悬停  ·  L = 就地降落   X = 停桨",
  teleop: "手动段  ·  WASD 平移  R/F 升降  Q/E 转向  空格 悬停  ·  L = 降落   X = 停桨",
  return_home: "返航中（遇到缓降请求会就地降落）", descend: "下降中", auto_land: "PX4 自动降落中，等落地上锁",
  cleanup: "飞行结束", handover: "飞手接管中 —— 脚本只看不动"};
const FLYING = new Set(["takeoff","hover","teleop","return_home","descend","auto_land","handover"]);
// ★2026-10-19 穿环: 新 stage 用 phase "gate" + 遥测 gate.seg 区分段;
//   如果它改用 gate_xxx 这种阶段名, 下面也认 (isGatePhase)
Object.assign(PHASE, {gate: "穿环", gate_wait: "等待识别", gate_align: "对准", gate_commit: "提交",
  gate_dash: "冲刺", gate_brake: "制动", gate_hold: "过环悬停"});
const SEG = {manual: "手动", wait: "等待识别", align: "对准", commit: "提交", dash: "冲刺", brake: "制动",
  hold: "过环悬停", abort: "中止"};
const isGatePhase = ph => typeof ph === "string" && (ph === "gate" || ph.startsWith("gate_"));
const isFlying = ph => FLYING.has(ph) || isGatePhase(ph);

function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
let C = {};
function loadColors() {
  for (const k of ["text","muted","dim","ok","warn","bad","info","tgt","sp","band","shell","grid","line","line2","panel2"])
    C[k] = css("--" + k);
}
// 画布尺寸跟着容器 (CSS 决定), 这里只按设备像素比对齐
function prep(cv) {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return {ctx, w, h};
}
function niceStep(x) {
  const p = Math.pow(10, Math.floor(Math.log10(x))); const m = x / p;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
}

// =====================================================================
// 全局状态
// =====================================================================
let S = null;            // /api/state
let CFG = null;          // /api/config → {source, config}
let cfgT = undefined;
let lastOkMs = 0;
let lastTelT = null;
let lastGateT = null;
let evLast = 0;
let activeTab = "mon";
let topMode = store("fm_topmode") || "fence";
let detWin = +(store("fm_detwin") || 60);
const trail = [];        // [t, x, y]  (相对原点)
// 历史曲线, 统一保留 HIST_MAX 秒 (监控页迷你曲线看 30s, 详情页可选 30/60/120s)
const HIST_MAX = 125;
const H = {d: [], ez: [], g: [], age: [], v: [], cv: [], z: [], spz: [], tgz: [],
           // ★穿环: 沿法向距离 / 靶面偏差 h,v / 沿法向速度 实测,指令 / 环位置 σ(cm) / 感知年龄(ms)
           ga: [], goh: [], gov: [], gva: [], gvc: [], gsd: [], gage: []};

// ---------------------------------------------------------------------
// 监控页用哪套配置画围栏 / 门限 / 安全余量 (★2026-10-12)
//   live  飞行脚本在跑 → 它发布的 /flight_ctrl/config (实际生效, 最优先)
//   ref   脚本没在跑 → 「起飞配置」页当前那套 (载入档案 / 改旋钮, 校验通过就跟着变)
//   last  脚本已退出、配置页也没动过 → 上次飞行生效的那套
//   default 代码默认值 (flight_monitor 启动时 dry-run 出来的)
// ---------------------------------------------------------------------
let REF = null;           // {params, derived, label, dirty}  —— 配置页最近一次校验通过的结果
function cfgSrc() {
  const c = CFG && CFG.config;
  const live = !!(CFG && CFG.source === "live" && c);
  if (live && S && S.script_alive) return {kind: "live", c};
  if (REF) return {kind: "ref", c: REF};
  if (c) return {kind: live ? "last" : "default", c};
  return null;
}
function P() { const s = cfgSrc(); return (s && s.c.params) || {}; }
function D() { const s = cfgSrc(); return (s && s.c.derived) || null; }
function tel() { return S && S.script_alive ? S.telemetry : null; }

// ---------------------------------------------------------------------
// 俯视图 / 高度条的原点 (★2026-10-12 重写)
//   · 飞行中: 飞行脚本报的原点 = preflight 通过那一刻飞机的位置 (起飞点)。
//     preflight 阶段及之前这个值还没定, 不用它。
//   · 脚本没在跑、飞机没解锁: 原点 = 飞机**当前**位置 = 预计起飞点。
//     (旧版这里会一直用上次飞行的原点, 直到重启 flight_monitor —— 挪过飞机 / 重启过
//      VINS 后围栏就画在一个毫无意义的地方)
//   · 脚本没在跑但飞机还解锁着 (脚本崩了 / 飞手接管后退出): 沿用最后一次飞行原点。
// ---------------------------------------------------------------------
const PRE_ORIGIN = new Set(["idle", "init", "preflight", "preflight_failed"]);
let lastFlightOrigin = null;
let ORIGIN_SRC = "none";        // flight | here | last | none
function origin() {
  const t = tel();
  if (t && !PRE_ORIGIN.has(t.phase)) {
    let o = t.origin && nz(t.origin[0]) ? t.origin : null;
    if (!o && CFG && CFG.source === "live" && CFG.config && CFG.config.origin
        && nz(CFG.config.origin[0])) o = CFG.config.origin;
    if (o) { lastFlightOrigin = o; ORIGIN_SRC = "flight"; return o; }
  }
  const armed = !!(S && S.mavros && S.mavros.armed);
  if (!t && armed && lastFlightOrigin) { ORIGIN_SRC = "last"; return lastFlightOrigin; }
  if (S && S.pose && S.pose.age < 2) { ORIGIN_SRC = "here"; return [S.pose.x, S.pose.y, S.pose.z]; }
  ORIGIN_SRC = "none";
  return null;
}
function rel(x, y, z) {
  const o = origin() || [0, 0, 0];
  return [x - o[0], y - o[1], z - o[2]];
}
const relv = p => rel(p[0], p[1], p[2]);
