"use strict";
// vb09/web/live.js —— /api/stream 增量 + 轮询 + 顶栏/提示行/历史
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// /api/stream: 遥测 + 环状态的增量, 一条不漏 (冲刺段遥测 20Hz, 只拉"最新一条"会丢大半)
//   老后端没有这个接口 (404) → 退回在 onState 里按 5Hz 取最新遥测
// =====================================================================
let lastGateCtlTm = 0;              // 最近一次收到脚本 gate 段 (控制数据) 的时刻
function ingestTel(tm, t) {
  if (!t) return;
  if (t.setpoint && t.setpoint.mode === "ZHOLD" && nz(t.setpoint.pz))
    push(H.spz, tm, rel(0, 0, t.setpoint.pz)[2]);
  const g = t.gate;
  if (g && nz(g.along)) {
    lastGateCtlTm = tm;
    push(H.ga, tm, g.along);
    if (Array.isArray(g.off)) { push(H.goh, tm, g.off[0]); push(H.gov, tm, g.off[1]); }
    push(H.gva, tm, g.v_along_meas); push(H.gvc, tm, g.v_along_cmd);
  }
}
function ingestGate(tm, s) {
  if (!s) return;
  if (gateStale(s)) { push(H.gage, tm, s.age_ms); return; }     // 旧估计: 只记年龄, 不画 σ / 偏差
  if (Array.isArray(s.sd_cm) && s.sd_cm.length && s.sd_cm.every(nz)) push(H.gsd, tm, Math.max(...s.sd_cm));
  push(H.gage, tm, s.age_ms);
  // 没有控制数据时用感知 + 当前位姿算 (只算刚到的条目: 回补的历史条目对不上当时的位姿)
  if (tm - lastGateCtlTm > 1 && S && S.now - tm < 0.6) {
    const g = mkGate("感知", vec3(s.target), vec3(s.n)), p = dronePos();
    if (g && g.n && p) {
      const q = toPlane(g, p);
      push(H.ga, tm, q.along); push(H.goh, tm, q.h); push(H.gov, tm, q.v);
      if (S.vel && S.vel.age < 1) push(H.gva, tm, -V3.dot([S.vel.x, S.vel.y, S.vel.z], g.n));
    }
  }
}
const STREAM = {since: 0, ok: null};
async function pollStream() {
  let wait = document.hidden ? 1000 : 150;
  try {
    const r = await fetch("/api/stream?since=" + STREAM.since, {cache: "no-store"});
    if (r.status === 404) { STREAM.ok = false; return; }
    const j = await r.json();
    STREAM.ok = true;
    if (j.seq < STREAM.since) STREAM.since = 0;          // 监控程序重启过, 序号从头来
    else {
      for (const e of j.tel || []) ingestTel(e.tm, e.d);
      for (const e of j.gate || []) ingestGate(e.tm, e.d);
      STREAM.since = j.seq;
    }
  } catch (e) { wait = 1000; }
  setTimeout(pollStream, wait);
}

// =====================================================================
// 轮询
// =====================================================================
async function getJSON(url, opt) {
  const r = await fetch(url, Object.assign({cache: "no-store"}, opt || {}));
  return r.json();
}
async function pollState() {
  try {
    S = await getJSON("/api/state");
    lastOkMs = Date.now();
    if (S.config_t !== cfgT) { cfgT = S.config_t; loadConfig(); }
    onState();
  } catch (e) { /* 连接断开: 由 linkTick 显示 */ }
  setTimeout(pollState, 200);
}
async function loadConfig() {
  try { CFG = await getJSON("/api/config"); } catch (e) {}
}
async function pollEvents() {
  try {
    const r = await getJSON("/api/events?since=" + evLast);
    for (const e of r.events) addEvent(e);
  } catch (e) {}
  setTimeout(pollEvents, 1000);
}
function linkTick() {
  const age = (Date.now() - lastOkMs) / 1000;
  const el = $("#link");
  if (!lastOkMs) { el.textContent = "连接中…"; el.className = "pill"; }
  else if (age > 2) { el.textContent = `与飞机断开 ${age.toFixed(0)}s`; el.className = "pill bad"; }
  else { el.textContent = "已连接"; el.className = "pill ok"; }
  document.body.classList.toggle("stale", !!lastOkMs && age > 2);
  if (lastOkMs && age > 2) updateHint(null);
  setTimeout(linkTick, 500);
}

// =====================================================================
// 状态 → 顶栏 + 提示行 + 历史
// =====================================================================
function setChip(id, text, cls, small) {
  const el = $(id);
  el.className = el.className.replace(/\bs-\w+/g, "").trim() + (cls ? " s-" + cls : "");
  el.querySelector("b").textContent = text;
  el.title = text + (small ? "  ·  " + small : "");
  const sm = el.querySelector("small"); if (sm) sm.textContent = small || "";
}

// 「开关」卡片 = 保险1 + 保险2 + Z环 (+ 配置名), 状态取三者里最严重的
const RANK = {bad: 4, warn: 3, info: 2, ok: 1, "": 0, dim: -1};
function chipCls(id) { const m = $(id).className.match(/\bs-(\w+)/); return m ? m[1] : ""; }
function mergeSwitchChip() {
  let best = null;
  for (const id of ["#c-loop", "#c-sim", "#c-z"]) {
    const c = chipCls(id);
    if (best === null || RANK[c] > RANK[best]) best = c;
  }
  const sim = chipCls("#c-sim") === "bad" ? "假数据可驱动!" : "SIM拦截";
  const txt = `${$("#c-loop b").textContent} · ${sim} · Z${$("#c-z b").textContent}`;
  setChip("#c-sw", txt, best || "", "配置 " + $("#c-cfg b").textContent);
}

// 状态条模式: 自动 (按宽度) / 全部 / 精简
const STRIP_LABEL = {auto: "自动", full: "全部", compact: "精简"};
let stripMode = STRIP_LABEL[store("fm_strip")] ? store("fm_strip") : "auto";
function applyStrip() {
  const el = $("#strip"), w = el.clientWidth;
  el.classList.toggle("c1", stripMode === "compact" || (stripMode === "auto" && w < 1150));
  el.classList.toggle("c2", stripMode !== "full" && w < 640);
  el.classList.toggle("c3", stripMode !== "full" && w < 480);
  const b = $("#strip-mode");
  b.textContent = STRIP_LABEL[stripMode];
  b.title = "状态条：" + STRIP_LABEL[stripMode] + "（点一下切换 自动 → 全部 → 精简）\n" +
    "精简 = 保险1/保险2/Z环/配置 合成一张「开关」；窗口很窄时再藏掉灰色卡片";
}
$("#strip-mode").addEventListener("click", () => {
  stripMode = {auto: "full", full: "compact", compact: "auto"}[stripMode];
  store("fm_strip", stripMode); applyStrip();
});
if (window.ResizeObserver) new ResizeObserver(applyStrip).observe($("#strip"));
else window.addEventListener("resize", applyStrip);
applyStrip();
const gotoCfg = () => document.querySelector('.tabs button[data-tab="cfg"]').click();
$("#c-cfg").addEventListener("click", gotoCfg);
$("#c-sw").addEventListener("click", gotoCfg);

function updateHint(t) {
  let cls = "dim", txt;
  const age = (Date.now() - lastOkMs) / 1000;
  if (!lastOkMs) txt = "连接中…";
  else if (age > 2) { cls = "bad"; txt = `与飞机上的监控服务断开 ${age.toFixed(0)}s —— 检查 SSH 端口映射 / flight_monitor.py`; }
  else if (t && t.abort) { cls = "bad"; txt = "★ FATAL 停桨：" + (t.abort_reason || ""); }
  else if (t && t.takeover) { cls = "warn";
    txt = `★ 控制权被接管（飞控模式 ${t.takeover}）—— 脚本不切模式、不停桨，等飞机落地上锁。只有 X 还有效`; }
  else if (t && t.land) { cls = "warn"; txt = "★ 缓降中：" + (t.land_reason || ""); }
  else if (!t) {
    if (S && S.pose && S.pose.age < 1) txt = "飞行脚本未运行  ·  起飞前看「机况检查」，配置在「起飞配置」，流程见「帮助」";
    else { cls = "warn"; txt = "没有位姿数据 —— mavros / VINS 没起来？（看「机况检查」）"; }
  } else if (isGatePhase(t.phase)) {
    // 穿环段: 文字只说"现在处于哪一段 / 还能不能中止", 具体按键以飞行终端里的提示为准
    const g = t.gate || {}, cm = g.commit || {}, seg = g.seg;
    const segTxt = SEG[seg] || seg || PHASE[t.phase] || t.phase;
    if (cm.passed || (cm.abort_ok === false)) {
      cls = "warn";
      txt = `穿环 · ${segTxt} · ★ 已过提交点: 不再中止, 只剩 X 停桨` +
            (t.shadow ? "（影子模式: 指令不下发）" : "");
    } else if (seg === "manual") {
      // ★2026-10-21 上一次为什么放弃 (比如"按设定 2.5 m/s 不可行 … 最高能冲 2.2") 直接写在提示行
      const pr = g.pre;
      if (pr && !pr.ok_full) {
        // ★2026-10-21 手动段实时预规划: 按 T 之前就知道放不放得下
        cls = "bad";
        txt = `穿环 · 手动段 · 按 ${fx(pr.v_set, 1)} m/s 放不下：${pr.why || "?"}` +
              (nz(pr.v_room) ? ` · 最高 ${fx(pr.v_room, 1)} m/s` : "") +
              (nz(pr.xy_need) ? ` · 满速要 xy_radius ≥ ${fx(pr.xy_need, 1)}` : "") + "   ·   挪位置 / 改档案，或照样按 T";
      } else if (g.why) { cls = "warn"; txt = "穿环 · 手动段 · 上一次放弃：" + g.why + "   ·   改好后按 T 再来 / L = 降落"; }
      else { cls = pr ? "ok" : "info";
             txt = (pr ? `穿环 · 手动段 · 按 ${fx(pr.v_set, 1)} m/s ✓ 放得下 —— ` : "穿环 · 手动段 —— ") +
                   "飞行终端 WASD / RF / QE 挪飞机, 到位后按 T 开始找环规划   ·   L = 降落   X = 停桨"; }
    } else if (seg === "wait") {
      cls = "info"; txt = "穿环 · 等待环识别收敛 —— 看「穿环」卡片的识别阶段和 σ   ·   M = 回手动   L = 降落   X = 停桨";
    } else if (seg === "abort") {
      cls = "warn"; txt = "穿环 · 已放弃，原地悬停后降落 —— 原因在飞行终端 / 事件流";
    } else if (seg === "commit") {
      cls = (g.plan || {}).slow ? "bad" : "warn";
      const pl = g.plan || {};
      if (pl.need_confirm) txt = `穿环 · ★ 降速：计划 ${fx(pl.v, 1)} m/s（设定 ${fx(pl.v_set, 1)}，${pl.slow_why || "场地限速"}）—— ` +
          `接受就在飞行终端按 Y 冲刺（G 不算）   ·   不接受按 M 回手动段   L = 降落   X = 停桨`;
      else txt = `穿环 · 已锁定，等确认（${gateSpeedTxt(t.gate || {}, gateCfg() || {})}）—— ★ 在飞行终端按 G 冲刺（之后不能中止）   ·   M = 不冲了，回手动   L = 降落   X = 停桨` +
            (t.shadow ? "（影子模式）" : "");
    } else {
      cls = t.shadow ? "info" : "ok";
      txt = `穿环 · ${segTxt}` + (nz(cm.d) ? `（距环 ${fx(cm.d, 1)} m 处提交）` : "") +
            (t.shadow ? " · 影子模式: 指令只计算不下发" : "") + "   ·   M = 回手动   L = 降落   X = 停桨";
    }
  } else if (t.phase === "preflight" && t.gate && t.gate.pre && !t.gate.pre.ok_full) {
    // ★2026-10-21 起飞前用地面看到的环先规划: 放不下设定速度 → 飞行终端在等 c / 退出
    const pr = t.gate.pre;
    cls = "bad";
    txt = `起飞前检查：按设定 ${fx(pr.v_set, 1)} m/s 放不下（${pr.why || "?"}）· 最高 ${fx(pr.v_room, 1)} m/s` +
          (nz(pr.xy_need) ? ` · 要满速需 xy_radius ≥ ${fx(pr.xy_need, 1)}` : "") + " —— 飞行终端里 c = 照样起飞 / 其它 = 退出改档案";
  } else txt = PHASE_HINT[t.phase] || (PHASE[t.phase] || t.phase);
  const el = $("#hint");
  el.className = cls;
  $("#hint-t").textContent = txt;
  el.title = txt;
}

function onState() {
  const t = tel();
  // 飞机主机名: 同时开两条隧道 (8080 / 8081) 时一眼看出这一页是哪架
  const tag = (S.host ? "· " + S.host : "") + (S.mode === "fake" ? " · FAKE 模拟数据" : "");
  if ($("#mode-tag").textContent !== tag) {
    $("#mode-tag").textContent = tag;
    document.title = "VB09 飞行监控" + (S.host ? " · " + S.host : "");
  }
  // 数据新鲜度: 脚本在跑看遥测, 否则看 mavros 位姿
  if (S.script_alive) {
    const a = S.telemetry_age;
    setChip("#c-data", a < 0.6 ? `实时 ${a.toFixed(1)}s` : `延迟 ${a.toFixed(1)}s`,
            a < 0.6 ? "ok" : "warn", "飞行脚本遥测");
  } else if (S.pose && S.pose.age < 1) {
    setChip("#c-data", "脚本未运行", "info",
            S.telemetry_age ? `上次遥测 ${S.telemetry_age.toFixed(0)}s 前` : "仅 mavros 数据");
  } else {
    setChip("#c-data", "无数据", "bad", S.pose ? `位姿 ${S.pose.age.toFixed(0)}s 前` : "mavros/VINS 没起?");
  }
  // 阶段 / 控制权
  if (t) {
    setChip("#c-phase", PHASE[t.phase] || t.phase,
            t.abort ? "bad" : t.land ? "warn" : isFlying(t.phase) ? "info" : "", t.phase);
    if (t.takeover) setChip("#c-sub", "飞手", "warn", t.takeover);
    else if (isGatePhase(t.phase)) {
      const g = t.gate || {}, cm = g.commit || {};
      const past = cm.passed || cm.abort_ok === false;
      setChip("#c-sub", (SEG[g.seg] || g.seg || "穿环") + (t.shadow ? "(影子)" : ""),
              past ? "warn" : t.shadow ? "info" : "ok", past ? "已过提交点" : "穿环段");
    }
    else setChip("#c-sub", isFlying(t.phase) ? "脚本/人工" : "—", "", "");
    setChip("#c-loop", t.shadow ? "影子模式" : "实闭环", t.shadow ? "ok" : "warn");
    setChip("#c-sim", t.sim_drive ? "假数据可驱动!" : "假数据拦截", t.sim_drive ? "bad" : "ok");
    setChip("#c-z", "—", "", "穿环没有 z 跟踪环");
  } else {
    setChip("#c-phase", "—", "dim", ""); setChip("#c-sub", "—", "dim", "");
    const p = P();
    const shadow = p.GATE_SHADOW, simd = p.GATE_ALLOW_SIM_DRIVE;
    setChip("#c-loop", shadow ? "影子模式" : "实闭环", "dim");
    setChip("#c-sim", simd ? "假数据可驱动!" : "假数据拦截", simd ? "bad" : "dim");
    setChip("#c-z", "—", "dim");
  }
  // 飞控
  const mv = S.mavros || {};
  if (!mv.connected) setChip("#c-fc", "未连接", "bad", "");
  else setChip("#c-fc", mv.armed ? "已解锁" : "未解锁", mv.armed ? "warn" : "", mv.mode || "");
  // 电池
  if (nz(S.batt_v) && S.batt_v > 0) {
    const cells = (t && t.cells) || (S.batt_v >= 18 ? 6 : 4);
    const flyMin = (t && t.fly_min_v) || (P().CELL_MIN_FLY || 3.5) * cells;
    const pc = S.batt_v / cells;
    const margin = S.batt_v - flyMin;
    setChip("#c-bat", `${S.batt_v.toFixed(2)} V`, margin < 0.5 ? "bad" : margin < 1.5 ? "warn" : "ok",
            `${cells}S ${pc.toFixed(2)}V/芯 · 距缓降 ${margin.toFixed(1)}V`);
  } else setChip("#c-bat", "—", "dim", "");
  // 配置来源 (点这个卡片 → 「起飞配置」页)
  const cs = cfgSrc();
  if (cs) {
    const name = c => (String(c.src || "").split("/").pop() || "默认值").replace(/\.conf$/, "");
    if (cs.kind === "live") setChip("#c-cfg", name(cs.c), "", "脚本实际生效");
    else if (cs.kind === "ref") setChip("#c-cfg", cs.c.label, cs.c.stale ? "warn" : cs.c.dirty ? "info" : "",
                                       cs.c.stale ? "配置页当前那套没通过校验 · 这里画的是上一套通过的"
                                                  : "配置页预览 · 起飞后以脚本实际为准");
    else if (cs.kind === "last") setChip("#c-cfg", "上次: " + name(cs.c), "dim",
                                        "上次飞行生效 · 点这里去配置页选这次的");
    else setChip("#c-cfg", "代码默认值", "dim", "点这里去配置页载入档案");
  }
  mergeSwitchChip();
  updateHint(t);

  // ---- 历史
  const now = S.now;
  if (S.pose && S.pose.age < 1) {
    const [x, y, z] = rel(S.pose.x, S.pose.y, S.pose.z);
    trail.push([now, x, y]);
    push(H.z, now, z);
  }
  while (trail.length && now - trail[0][0] > 10) trail.shift();
  if (S.vel && S.vel.age < 1) push(H.v, now, Math.hypot(S.vel.x, S.vel.y));
  // 遥测 / 环状态历史正常由 pollStream 增量灌入 (一条不漏); 老后端没有 /api/stream 时退回这里按 5Hz 取
  if (STREAM.ok === false) {
    if (t && t.t !== lastTelT) { lastTelT = t.t; ingestTel(now, t); }
    if (S.gate && S.gate.t !== lastGateT) { lastGateT = S.gate.t; ingestGate(now, S.gate); }
  }
  for (const k in H) while (H[k].length && now - H[k][0][0] > HIST_MAX) H[k].shift();

  const armed = !!mv.armed || !!(t && t.air);
  $("#cfg-armed").hidden = !armed;
  setSaveEnabled(!(armed || !lastPreviewOk));

  if (activeTab === "mon") renderMonitor();
  else if (activeTab === "det") renderDetail();
}
function push(arr, t, v) { if (nz(v)) arr.push([t, v]); }
