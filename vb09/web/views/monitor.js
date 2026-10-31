"use strict";
// vb09/web/views/monitor.js —— 监控页: 俯视图 / 姿态球 / 高度条 / 安全余量 / 穿环卡片 / 曲线
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 监控页
// =====================================================================
function renderMonitor() {
  if (!S) return;
  loadColors();
  ATT.drawn = undefined;          // 尺寸 / 主题变化后让姿态球在下一帧重画
  const t = tel();
  const d = D();
  const o = origin();
  $("#top-info").textContent = {
    flight: "原点 = 本次起飞点",
    here: "原点 = 飞机当前位置（预计起飞点）",
    last: "原点 = 最后一次起飞点（脚本已退出）",
  }[ORIGIN_SRC] || "无位姿 · 世界坐标";

  let drone = null;
  if (S.pose && S.pose.age < 2) {
    const [x, y, z] = rel(S.pose.x, S.pose.y, S.pose.z);
    drone = {x, y, z, yaw: S.pose.yaw};
  }
  let sp = null, spz = null;
  if (t && t.setpoint && (t.setpoint.mode === "ZHOLD" || t.setpoint.mode === "TRAJ") && nz(t.setpoint.px)) {
    const r = rel(t.setpoint.px, t.setpoint.py, t.setpoint.pz); sp = r; spz = r[2];
  }
  const lim = (t && t.envelope) ? t.envelope[0] : null;
  // ★穿环: 环 / 计划路径 / 制动段画进俯视图和高度条
  const G = gateNow();
  const gtop = G ? topGateData(G) : null;
  drawTop($("#cv-top"), {d, drone, sp, trail, lim, mode: topMode, gate: gtop});
  let gh = null;
  if (G && G.geo) gh = {cz: relv(G.geo.c)[2], r_in: G.geo.r_in, r_out: G.geo.r_out,
                        aimz: relv(G.geo.aim)[2], vertical: !!G.geo.n};
  drawHeight($("#cv-h"), {d, z: drone ? drone.z : null, spz, gate: gh});
  renderSafety(t);
  renderGate(t, G);
  drawCamOverlay();
}

// 俯视图要画的穿环元素 (全部换成相对原点)
function topGateData(G) {
  const out = {rings: [], plan: null, brake: null, pred: null, cross: null,
               corridor: nz((gateCfg() || {}).d_align) ? +gateCfg().d_align + 1.0 : 4.0};
  const relG = (g, color) => Object.assign({}, g, {c: relv(g.c), aim: relv(g.aim), color});
  const same = G.used && G.per && G.used.n && G.per.n &&
               Math.hypot(...V3.sub(G.used.c, G.per.c)) < 0.02;
  if (G.per && !same) out.rings.push(relG(G.per, C.tgt));
  if (G.used) out.rings.push(relG(G.used, C.sp));
  const tg = G.tg || {};
  if (tg.plan && Array.isArray(tg.plan.pts)) {
    const pts = tg.plan.pts.map(vec3);
    if (pts.every(Boolean)) out.plan = {pts: pts.map(relv), labels: tg.plan.labels || []};
  }
  if (G.geo && G.geo.n) {
    out.cross = relv(G.pred ? planeToWorld(G.geo, G.pred[0], G.pred[1]) : G.geo.aim);
    if (G.pred) out.pred = out.cross;
  }
  if (tg.brake && vec3(tg.brake.end)) out.brake = {end: relv(vec3(tg.brake.end))};
  return out;
}

// ---------------- 俯视图 ----------------
function drawTop(cv, o) {
  const {ctx, w, h} = prep(cv);
  if (w < 20 || h < 20) return;
  const d = o.d;
  const hard = d ? d.xy_hard : null, soft = d ? d.xy_soft : null;
  let cx = 0, cy = 0, R, zoomed = false;
  if (o.mode === "follow" && o.drone) { cx = o.drone.x; cy = o.drone.y; R = 3.2; }
  else {
    R = Math.max(hard || 2, 1.5);
    const pts = [o.drone];
    if (o.gate) {                         // 穿环: 环 / 计划路径 / 制动终点都要在画面里
      for (const g of o.gate.rings) pts.push({x: g.c[0], y: g.c[1]});
      for (const q of (o.gate.plan ? o.gate.plan.pts : [])) pts.push({x: q[0], y: q[1]});
      if (o.gate.brake) pts.push({x: o.gate.brake.end[0], y: o.gate.brake.end[1]});
    }
    for (const p of pts) if (p) R = Math.max(R, Math.hypot(p.x, p.y) + 0.8);
    // 穿环: 围栏比环附近的范围大很多时 (常见: 围栏 10 m, 环在 5 m 外), 只框 原点 + 飞机 + 环 + 计划 + 制动终点,
    //   不然环和计划路径挤成一团。围栏没比它大多少就照旧以原点为中心把整个围栏框进来
    if (o.gate && (o.gate.rings.length || o.gate.plan)) {
      let x0 = 0, x1 = 0, y0 = 0, y1 = 0;
      const ext = (x, y) => { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); };
      for (const p of pts) if (p) ext(p.x, p.y);
      for (const g of o.gate.rings) if (g.n) for (const k of [-g.r_out, g.r_out])
        ext(g.c[0] + g.right[0] * k, g.c[1] + g.right[1] * k);
      const Rg = Math.max(x1 - x0, y1 - y0) / 2 + 1.0;
      if (!(hard && hard <= Rg * 1.6)) { cx = (x0 + x1) / 2; cy = (y0 + y1) / 2; R = Rg; zoomed = true; }
    }
    R *= 1.05;
  }
  const s = Math.min(w, h) / 2 / R;
  const X = x => w / 2 + (x - cx) * s, Y = y => h / 2 - (y - cy) * s;
  const ring = (x, y, r, color, dash, lw) => {
    ctx.beginPath(); ctx.setLineDash(dash || []); ctx.strokeStyle = color; ctx.lineWidth = lw || 1;
    ctx.arc(X(x), Y(y), Math.max(0, r * s), 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
  };
  // 网格
  const step = niceStep(R / 3.5);
  ctx.font = "11px " + css("--mono"); ctx.fillStyle = C.dim; ctx.textAlign = "left";
  const far = R * 1.5 + Math.hypot(cx, cy);
  for (let r = step; r < far; r += step) {
    ring(0, 0, r, C.grid, null, 1);
    ctx.fillText(+r.toFixed(2) + "m", X(0) + r * s * 0.707 + 3, Y(0) - r * s * 0.707 - 2);
  }
  ctx.strokeStyle = C.grid; ctx.beginPath();
  ctx.moveTo(X(0), 0); ctx.lineTo(X(0), h); ctx.moveTo(0, Y(0)); ctx.lineTo(w, Y(0)); ctx.stroke();
  ctx.fillStyle = C.muted; ctx.fillText("北", X(0) + 4, 12); ctx.fillText("东", w - 16, Y(0) - 4);
  // 围栏
  if (soft) ring(0, 0, soft, C.muted, [6, 5], 1.2);
  if (hard) ring(0, 0, hard, C.bad, null, 1.4);
  if (nz(o.lim) && o.lim < (hard || 1e9) - 0.01) ring(0, 0, o.lim, C.warn, [4, 4], 1.4);
  // 原点
  ctx.strokeStyle = C.text; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(X(0) - 6, Y(0)); ctx.lineTo(X(0) + 6, Y(0));
  ctx.moveTo(X(0), Y(0) - 6); ctx.lineTo(X(0), Y(0) + 6); ctx.stroke();
  // ★穿环: 环 (从上面看是一条线) / 进近走廊 / 法向 / 红点 / 计划路径 / 制动段 / 预计穿越点
  if (o.gate) {
    const G = o.gate;
    const seg2 = (a, b, color, lw, dash, alpha) => {
      ctx.beginPath(); ctx.setLineDash(dash || []); ctx.strokeStyle = color; ctx.lineWidth = lw;
      ctx.globalAlpha = alpha || 1;
      ctx.moveTo(X(a[0]), Y(a[1])); ctx.lineTo(X(b[0]), Y(b[1])); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1;
    };
    for (const g of G.rings) {
      const c = g.c;
      if (!g.n) {                         // 没有法向: 只能画中心点
        ctx.strokeStyle = g.color; ctx.lineWidth = 2; ctx.beginPath();
        ctx.arc(X(c[0]), Y(c[1]), 7, 0, Math.PI * 2); ctx.stroke();
        ctx.fillStyle = g.color; ctx.fillText(g.kind + "·环中心（无法向）", X(c[0]) + 9, Y(c[1]) - 8);
        continue;
      }
      const r = g.right, n = g.n;
      const at = (k, m) => [c[0] + r[0] * k + n[0] * (m || 0), c[1] + r[1] * k + n[1] * (m || 0)];
      seg2(at(-g.r_in), at(-g.r_in, G.corridor), g.color, 1, [3, 4], 0.7);
      seg2(at(g.r_in), at(g.r_in, G.corridor), g.color, 1, [3, 4], 0.7);
      seg2(at(-g.r_out), at(-g.r_in), g.color, 5);
      seg2(at(g.r_in), at(g.r_out), g.color, 5);
      seg2(at(-g.r_in), at(g.r_in), g.color, 1);
      const tip = at(0, 0.9);              // 法向箭头, 指向进近侧
      seg2(at(0), tip, g.color, 1.5);
      const ang = Math.atan2(-(n[1]), n[0]);
      ctx.save(); ctx.translate(X(tip[0]), Y(tip[1])); ctx.rotate(ang);
      ctx.fillStyle = g.color; ctx.beginPath(); ctx.moveTo(6, 0); ctx.lineTo(-4, 4); ctx.lineTo(-4, -4);
      ctx.closePath(); ctx.fill(); ctx.restore();
      if (g.sd && g.sd[0] * s > 3) ring(c[0], c[1], g.sd[0], g.color, [2, 3], 1);
      ctx.fillStyle = "#e5534b"; ctx.beginPath();
      ctx.arc(X(g.aim[0]), Y(g.aim[1]), 3.5, 0, Math.PI * 2); ctx.fill();
      const lab = at(g.r_out + 0.1, -0.35);
      ctx.fillStyle = g.color;
      ctx.fillText(g.kind + (g.locked ? "·锁定" : ""), X(lab[0]), Y(lab[1]));
    }
    if (G.plan && G.plan.pts.length > 1) {
      ctx.beginPath(); ctx.setLineDash([6, 4]); ctx.strokeStyle = C.sp; ctx.lineWidth = 1.5;
      G.plan.pts.forEach((q, i) => i ? ctx.lineTo(X(q[0]), Y(q[1])) : ctx.moveTo(X(q[0]), Y(q[1])));
      ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = C.sp;
      G.plan.pts.forEach((q, i) => {
        ctx.fillRect(X(q[0]) - 2.5, Y(q[1]) - 2.5, 5, 5);
        // 离环很近的点 (穿越点) 不写字: 环自己有标签, 挤在一起谁都看不清
        const nearRing = G.rings.some(g => Math.hypot(q[0] - g.c[0], q[1] - g.c[1]) < 0.6);
        if (G.plan.labels[i] && !nearRing) ctx.fillText(G.plan.labels[i], X(q[0]) + 5, Y(q[1]) + 13);
      });
    }
    if (G.brake && G.cross) {
      const e = G.brake.end, out = hard && Math.hypot(e[0], e[1]) > hard;
      seg2(G.cross, e, out ? C.bad : C.warn, 4, null, 0.75);
      ctx.fillStyle = out ? C.bad : C.warn; ctx.beginPath();
      ctx.arc(X(e[0]), Y(e[1]), 4, 0, Math.PI * 2); ctx.fill();
      if (out) ctx.fillText("制动终点出硬围栏!", X(e[0]) + 6, Y(e[1]) - 6);
    }
    if (G.pred) {
      const px = X(G.pred[0]), py = Y(G.pred[1]);
      ctx.strokeStyle = C.sp; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(px - 5, py - 5); ctx.lineTo(px + 5, py + 5); ctx.moveTo(px + 5, py - 5); ctx.lineTo(px - 5, py + 5);
      ctx.stroke();
    }
  }
  // 轨迹 (最近 10s)
  if (o.trail && o.trail.length > 1) {
    const tN = o.trail[o.trail.length - 1][0];
    ctx.lineWidth = 1.5;
    for (let i = 1; i < o.trail.length; i++) {
      const a = o.trail[i - 1], b = o.trail[i];
      ctx.strokeStyle = C.info; ctx.globalAlpha = clamp(1 - (tN - b[0]) / 10, 0.05, 0.8);
      ctx.beginPath(); ctx.moveTo(X(a[1]), Y(a[2])); ctx.lineTo(X(b[1]), Y(b[2])); ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }
  if (o.drone) {
    if (o.sp) {
      const L = Math.hypot(o.sp[0] - o.drone.x, o.sp[1] - o.drone.y);
      const lm = o.leashMax || P().GATE_REPO_LEASH || 0.6;
      ctx.strokeStyle = L > 0.9 * lm ? C.warn : C.sp; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(X(o.drone.x), Y(o.drone.y)); ctx.lineTo(X(o.sp[0]), Y(o.sp[1])); ctx.stroke();
      ctx.strokeRect(X(o.sp[0]) - 4, Y(o.sp[1]) - 4, 8, 8);
    }
    const px = X(o.drone.x), py = Y(o.drone.y), a = -o.drone.yaw;
    ctx.save(); ctx.translate(px, py); ctx.rotate(a);
    ctx.fillStyle = C.text; ctx.beginPath();
    ctx.moveTo(13, 0); ctx.lineTo(-8, 8); ctx.lineTo(-4, 0); ctx.lineTo(-8, -8); ctx.closePath(); ctx.fill();
    ctx.restore();
    if (!o.noLabel) {
      ctx.fillStyle = C.muted; ctx.font = "11px " + css("--mono");
      const lab = `(${o.drone.x.toFixed(2)}, ${o.drone.y.toFixed(2)})  r=${Math.hypot(o.drone.x, o.drone.y).toFixed(2)}m`;
      const tw = ctx.measureText(lab).width;
      ctx.fillText(lab, clamp(px + 14, 2, w - tw - 2), clamp(py + 16, 12, h - 4));
    }
  }
  // 比例尺
  const sb = niceStep(R / 3);
  ctx.strokeStyle = C.muted; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(12, h - 14); ctx.lineTo(12 + sb * s, h - 14); ctx.stroke();
  ctx.fillStyle = C.muted; ctx.fillText(+sb.toFixed(2) + " m", 14, h - 20);
  if (!d) { ctx.fillStyle = C.dim; ctx.fillText("没有配置数据 —— 围栏未知", 12, 16); }
  else if (zoomed) {
    ctx.fillStyle = C.dim; ctx.textAlign = "right";
    ctx.fillText(`已放大到环附近 · 硬围栏 ${fx(hard, 1)} m 可能在画外`, w - 8, h - 6);
    ctx.textAlign = "left";
  }
}

// ---------------- 姿态球 ----------------
// a = {roll, pitchUp, tilt} (度; roll 右侧下为正, pitchUp 机头上仰为正, 已在 flight_monitor 换好符号)
function drawAtt(cv, a, maxTilt) {
  const {ctx, w, h} = prep(cv);
  const R = Math.min(w, h - 30) / 2 - 3;
  if (R < 20) return;
  const cx = w / 2, cy = R + 3, D2R = Math.PI / 180, ppd = R / 40;   // 半径 = ±40° 俯仰
  const mono = css("--mono");
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.clip();
  if (!a) {
    ctx.fillStyle = C.panel2; ctx.fillRect(0, 0, w, h); ctx.restore();
    ctx.strokeStyle = C.line2; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = C.dim; ctx.font = "11px " + mono; ctx.textAlign = "center";
    ctx.fillText("无姿态", cx, cy + 4); return;
  }
  // 天地: 飞机右滚 → 地平线逆时针转; 机头上仰 → 地平线下移
  ctx.translate(cx, cy); ctx.rotate(-a.roll * D2R); ctx.translate(0, a.pitchUp * ppd);
  ctx.fillStyle = "#2f6fb5"; ctx.fillRect(-2 * R, -4 * R, 4 * R, 4 * R);
  ctx.fillStyle = "#7a5230"; ctx.fillRect(-2 * R, 0, 4 * R, 4 * R);
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(-2 * R, 0); ctx.lineTo(2 * R, 0); ctx.stroke();
  ctx.lineWidth = 1; ctx.fillStyle = "#fff"; ctx.font = "9px " + mono; ctx.textAlign = "left";
  for (let p = -30; p <= 30; p += 10) {                 // 俯仰梯
    if (!p) continue;
    const y = -p * ppd, half = R * (p % 20 ? 0.16 : 0.28);
    ctx.beginPath(); ctx.moveTo(-half, y); ctx.lineTo(half, y); ctx.stroke();
    ctx.fillText(String(Math.abs(p)), half + 3, y + 3);
  }
  ctx.restore();

  // 外圈按合成倾角变色 (和安全余量同一个限值: MAX_TILT_DEG 持续超限 = 空中停桨)
  const col = a.tilt > maxTilt * 0.83 ? C.bad : a.tilt > maxTilt * 0.5 ? C.warn : C.line2;
  ctx.strokeStyle = col; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.stroke();
  const tick = (t, len, color, lw) => {
    const s = Math.sin(t * D2R), c = Math.cos(t * D2R);
    ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.beginPath();
    ctx.moveTo(cx + (R - len) * s, cy - (R - len) * c); ctx.lineTo(cx + R * s, cy - R * c); ctx.stroke();
  };
  for (const t of [-60, -45, -20, -10, 0, 10, 20, 45, 60]) tick(t, t % 30 ? 6 : 10, "#fff", 1.2);
  tick(-maxTilt, 12, C.bad, 2.5); tick(maxTilt, 12, C.bad, 2.5);   // 停桨线

  // 横滚指针 (跟天空一起转, 对着外圈固定刻度读数)
  ctx.save(); ctx.translate(cx, cy); ctx.rotate(-a.roll * D2R);
  ctx.fillStyle = "#fff"; ctx.beginPath();
  ctx.moveTo(0, -R + 12); ctx.lineTo(-5, -R + 20); ctx.lineTo(5, -R + 20); ctx.closePath(); ctx.fill();
  ctx.restore();
  // 固定的飞机符号
  ctx.strokeStyle = "#ffd33d"; ctx.lineWidth = 3; ctx.beginPath();
  ctx.moveTo(cx - R * 0.5, cy); ctx.lineTo(cx - R * 0.15, cy); ctx.lineTo(cx, cy + R * 0.1);
  ctx.lineTo(cx + R * 0.15, cy); ctx.lineTo(cx + R * 0.5, cy); ctx.stroke();

  ctx.font = "11px " + mono; ctx.textAlign = "center";
  const sg = v => (v >= 0 ? "+" : "") + v.toFixed(1);
  ctx.fillStyle = C.text; ctx.fillText(`R${sg(a.roll)} P${sg(a.pitchUp)}`, cx, cy + R + 14, w - 2);
  ctx.fillStyle = col === C.line2 ? C.muted : col;
  ctx.fillText(`倾角 ${a.tilt.toFixed(1)}°/${maxTilt}`, cx, cy + R + 27, w - 2);
}

// 轮询是 5Hz, 用 rAF 插值让球转得顺 (只是看着顺, 不会更及时)。
// 只在监控页可见时画; 数据没变化且已经收敛就不重画, 省 CPU。
const ATT = {tgt: null, cur: null, drawn: undefined};
function attLoop() {
  if (activeTab === "mon" && !document.hidden && S) {
    const p = S.pose;
    const t = (p && p.age < 1 && nz(p.roll) && nz(p.pitch_up))
      ? {roll: p.roll, pitchUp: p.pitch_up, tilt: p.tilt} : null;
    let moving = false;
    if (!t) ATT.cur = null;
    else if (!ATT.cur) { ATT.cur = Object.assign({}, t); moving = true; }
    else {
      const dr = t.roll - ATT.cur.roll, dp = t.pitchUp - ATT.cur.pitchUp;
      moving = Math.abs(dr) > 0.02 || Math.abs(dp) > 0.02 || ATT.cur.tilt !== t.tilt;
      ATT.cur.roll += dr * 0.3; ATT.cur.pitchUp += dp * 0.3; ATT.cur.tilt = t.tilt;
    }
    const key = ATT.cur ? "on" : "off";
    if (moving || ATT.drawn !== key) {
      drawAtt($("#cv-att"), ATT.cur, P().MAX_TILT_DEG || 30);
      ATT.drawn = key;
    }
  }
  requestAnimationFrame(attLoop);
}

// ---------------- 高度条 ----------------
function drawHeight(cv, o) {
  const {ctx, w, h} = prep(cv);
  if (w < 20 || h < 20) return;
  const d = o.d;
  ctx.font = "11px " + css("--mono");
  // 派生量不全 (flight_profile.py 不在 / dry-run 失败) 时 d 可能是 {} —— 一起当"无配置"
  if (!d || !d.z_band || !d.z_clamp || !nz(d.max_height)) {
    ctx.fillStyle = C.dim; ctx.fillText("无配置", 4, 16); return;
  }
  const gz = o.gate && nz(o.gate.cz) ? o.gate : null;
  const top = Math.max(d.max_height + 0.6, nz(o.z) ? o.z + 0.6 : 0,
                       gz ? gz.cz + gz.r_out + 0.4 : 0, 2.2);
  const pad = 12, x0 = 36, x1 = w - 34;
  const Y = z => pad + (1 - z / top) * (h - 2 * pad);
  if (gz) {                               // ★穿环: 淡紫 = 环外径范围, 深一点 = 开口, 红线 = 红点高度
    ctx.fillStyle = C.tgt;
    if (gz.vertical) {
      ctx.globalAlpha = 0.18;
      ctx.fillRect(x0, Y(gz.cz + gz.r_out), x1 - x0, Y(gz.cz - gz.r_out) - Y(gz.cz + gz.r_out));
      ctx.globalAlpha = 0.32;
      ctx.fillRect(x0, Y(gz.cz + gz.r_in), x1 - x0, Y(gz.cz - gz.r_in) - Y(gz.cz + gz.r_in));
      ctx.globalAlpha = 1;
    }
    ctx.strokeStyle = "#e5534b"; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo((x0 + x1) / 2 - 12, Y(gz.aimz)); ctx.lineTo((x0 + x1) / 2 + 12, Y(gz.aimz)); ctx.stroke();
  }
  ctx.textAlign = "right";
  const step = niceStep(top / Math.max(3, h / 70));
  ctx.strokeStyle = C.grid; ctx.fillStyle = C.dim;
  for (let z = 0; z <= top; z += step) {
    ctx.beginPath(); ctx.moveTo(x0, Y(z)); ctx.lineTo(x1, Y(z)); ctx.stroke();
    ctx.fillText(z.toFixed(step < 1 ? 1 : 0), x0 - 4, Y(z) + 4);
  }
  ctx.fillStyle = C.band; ctx.fillRect(x0, Y(d.z_band[1]), x1 - x0, Y(d.z_band[0]) - Y(d.z_band[1]));
  const bw = x1 - x0 - 4;
  const hline = (z, color, dash, lw) => {
    ctx.strokeStyle = color; ctx.lineWidth = lw || 1.2; ctx.setLineDash(dash || []);
    ctx.beginPath(); ctx.moveTo(x0, Y(z)); ctx.lineTo(x1, Y(z)); ctx.stroke(); ctx.setLineDash([]);
  };
  hline(0, C.text, null, 1.5);
  hline(d.z_clamp[0], C.muted, [3, 3]);
  hline(d.z_clamp[1], C.muted, [3, 3]);
  hline(d.max_height, C.bad, null, 1.8);
  // 标签按重要性放, 和已放的标签挨得太近 (<12px) 就不画 —— 窄条里不重叠
  const placed = [];
  const label = (y, text, color) => {
    if (y < 10 || y > h - 1 || placed.some(p => Math.abs(p - y) < 12)) return;
    placed.push(y);
    ctx.fillStyle = color; ctx.textAlign = "left"; ctx.fillText(text, x0 + 2, y, bw);
  };
  label(Y(0) - 3, "地面", C.text);
  label(Y(d.max_height) - 3, "硬顶 " + d.max_height.toFixed(1), C.bad);
  if (gz) label(Y(gz.cz + gz.r_out) - 3, `环 ${gz.cz.toFixed(2)}`, C.tgt);
  label(Y(d.z_band[1]) + 12, `门限带 ${d.z_band[0].toFixed(1)}~${d.z_band[1].toFixed(1)}`, C.info);
  label(Y(d.z_clamp[1]) - 3, "钳位 " + d.z_clamp[1].toFixed(1), C.muted);
  const tri = (z, color, filled, dir) => {
    const y = Y(clamp(z, 0, top)), x = dir > 0 ? x1 + 2 : x0 - 2;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + 10 * dir, y - 6); ctx.lineTo(x + 10 * dir, y + 6); ctx.closePath();
    if (filled) { ctx.fillStyle = color; ctx.fill(); } else { ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke(); }
  };
  if (nz(o.spz)) tri(o.spz, C.sp, false, 1);
  if (nz(o.z)) {
    tri(o.z, C.text, true, 1);
    ctx.fillStyle = C.text; ctx.textAlign = "left"; ctx.fillText(o.z.toFixed(2), x1 + 13, Y(clamp(o.z, 0, top)) + 4, w - x1 - 14);
  }
}

// ---------------- 安全余量 ----------------
function renderSafety(t) {
  const p = P(), d = D() || {};
  // 脚本的 safety_monitor 只在 preflight 之后才跑; 之前(或它停了)用网页估算
  const sf = t && t.safety && Object.keys(t.safety).length &&
             (!nz(t.safety.t) || Math.abs(t.t - t.safety.t) < 1.5) ? t.safety : null;
  const mv = S.mavros || {};
  let src, v = {};
  if (sf) { v = sf; src = "飞行脚本实时判据"; }
  else {
    src = "网页估算（脚本未在监控）";
    const o = origin();
    if (S.pose) {
      const [x, y, z] = rel(S.pose.x, S.pose.y, S.pose.z);
      v.dz = z; v.horiz = Math.hypot(x, y); v.tilt = S.pose.tilt; v.pose_age = S.pose.age;
    }
    if (S.vel) v.horiz_v = Math.hypot(S.vel.x, S.vel.y);
    v.odom_age = S.ages ? S.ages.vins : null;
    v.batt_v = S.batt_v;
    v.max_tilt = p.MAX_TILT_DEG; v.max_height = p.MAX_HEIGHT;
    const pe = d.phase_envelope || {};
    const env = (t && t.envelope) || pe.hover || pe._default || [null, null];
    v.horiz_lim = env[0]; v.vel_lim = env[1];
    v.pose_stale = p.POSE_STALE_SEC; v.vision_lost = p.VISION_LOST_LAND_S;
    v.jump_max = p.POSE_JUMP_MAX; v.deb = {};
    if (!o) v.horiz = null;
  }
  $("#saf-src").textContent = src;
  const cells = (t && t.cells) || (nz(S.batt_v) && S.batt_v >= 18 ? 6 : 4);
  const flyMin = v.fly_min_v || (p.CELL_MIN_FLY || 3.5) * cells;
  const full = 4.2 * cells;
  const deb = v.deb || {};
  const armed = !!mv.armed;
  const rows = [
    ["倾角", "停桨", v.tilt, v.max_tilt, "°", 1, deb.tilt],
    ["高度", "缓降", v.dz, v.max_height, "m", 2, deb.height],
    ["水平", "缓降", v.horiz, v.horiz_lim, "m", 2, deb.horiz],
    ["水平速度", "缓降", v.horiz_v, v.vel_lim, "m/s", 2],
    ["位姿延迟", "缓降", nz(v.pose_age) ? v.pose_age * 1000 : null, nz(v.pose_stale) ? v.pose_stale * 1000 : null, "ms", 0],
    ["VINS断供", armed ? "缓降" : "未解锁", nz(v.odom_age) ? v.odom_age * 1000 : null, nz(v.vision_lost) ? v.vision_lost * 1000 : null, "ms", 0, null, !armed],
    ["位姿跳变", "缓降", nz(v.jump) ? v.jump * 100 : null, nz(v.jump_max) ? v.jump_max * 100 : null, "cm", 1],
  ];
  let html = rows.map(([lbl, kind, cur, lim, u, dp, n, na]) => {
    const ok = nz(cur) && nz(lim) && lim > 0;
    const f = ok ? cur / lim : 0;
    const cls = !ok ? "" : f >= 1 ? "bad" : f >= 0.8 ? "warn" : "";
    const obs = n > 0 ? " obs" : "";
    return `<div class="srow${ok && !na ? "" : " na"}"><div class="lbl">${lbl}<em>${kind}</em></div>
      <div class="bar ${cls}${obs}" title="${n > 0 ? "去抖观察期中" : ""}"><i style="width:${(clamp(f, 0, 1) * 100).toFixed(1)}%"></i></div>
      <div class="val num">${fx(cur, dp)} / ${fx(lim, dp)} ${u}</div></div>`;
  }).join("");
  // 电压: 满电 → 0, 到缓降门槛 → 1
  const okV = nz(v.batt_v) && v.batt_v > 0;
  const fV = okV ? clamp((full - v.batt_v) / (full - flyMin), 0, 1.2) : 0;
  html += `<div class="srow${okV ? "" : " na"}"><div class="lbl">电压<em>缓降</em></div>
    <div class="bar ${fV >= 1 ? "bad" : fV >= 0.8 ? "warn" : ""}"><i style="width:${(clamp(fV, 0, 1) * 100).toFixed(1)}%"></i></div>
    <div class="val num">${fx(v.batt_v, 2)} / ${fx(flyMin, 1)} V</div></div>`;
  $("#saf-rows").innerHTML = html;
}

// ---------------- 卡片数字格 ----------------
function setTile(id, val, cls, sub) {
  const el = $(id);
  el.className = el.className.replace(/\b(ok|warn|bad)\b/g, "").trim() + (cls ? " " + cls : "");
  el.querySelector("b").textContent = val;
  const sm = el.querySelector("small"); sm.textContent = sub || ""; sm.title = sub || "";
}

// ---------------- 穿环卡片 (★2026-10-19) ----------------
// 冲刺速度: 有计划用计划里的实际速度 (场地放不下会被降速), 和设定值不同就标出来
function gateSpeedTxt(tg, gc) {
  const set = tg && tg.plan && nz(tg.plan.v_set) ? tg.plan.v_set : nz(P().DASH_V) ? P().DASH_V : gc.dash_v;
  const pl = tg && tg.plan && nz(tg.plan.v) ? tg.plan.v : null;
  if (nz(pl)) return pl < set - 1e-6 ? `★ 降速：计划 ${fx(pl, 1)} m/s（设定 ${fx(set, 1)}，${tg.plan.slow_why || "场地限速"}）` : `计划 ${fx(pl, 1)} m/s`;
  if (nz(gc.v_plan) && nz(set) && gc.v_plan < set - 1e-6) return `预计 ${fx(gc.v_plan, 1)} m/s（环后空间限速，设定 ${fx(set, 1)}）`;
  return nz(set) ? `冲刺 ${fx(set, 1)} m/s` : "";
}
// 阶段名中文 GATE_STAGE 定义在下面「识别画面」那一节 (画面左下角数字也用它)
const cm =(v, d = 0) => nz(v) ? (v * 100 >= 0 ? "+" : "") + (v * 100).toFixed(d) : "—";
function renderGate(t, G) {
  const s = S.gate, g = G && G.geo, tg = (G && G.tg) || {};
  const gc = gateCfg() || {};
  $("#card-gate").classList.toggle("idle", !g);
  let head = !s ? "没有环识别数据（/gate/status）" : s.age > 3 ? `环数据 ${fx(s.age, 0)} s 前` :
             g && !g.n ? "没有法向 n，只能显示环中心" :
             // ★飞行脚本: 识别没给 n 时, 脚本用"−进入方向"或视线方向代替 (n_src), 靶面按它画
             g && g.n_src && g.n_src !== "bridge" ? "法向按进入方向估计（识别没给 n）" : "";
  if (G && G.src) head += `<span class="src-tag ${G.src === "控制" ? "ctl" : "per"}">${G.src}</span>`;
  if (G && G.used) head += `<span class="src-tag ctl">${G.used.locked ? "环已锁定" : "环未锁定"}</span>`;
  const pl = tg.plan || {};
  if (pl.slow) head += `<span class="src-tag" style="border-color:var(--bad);color:var(--bad)" title="${esc(pl.slow_why || "")}">` +
                       `降速 ${fx(pl.v, 1)}/${fx(pl.v_set, 1)} m/s</span>`;
  $("#gate-src").innerHTML = head;

  // 识别 (感知, bridge)
  if (!s) setTile("#g-stage", "—", "", "");
  else if (gateStale(s)) setTile("#g-stage", "无新检测", "",
            `环估计 ${fx(s.age_ms / 1000, 1)} s 没更新` + (s.age_ms > 5000 ? " · 桥没重置? (新版桥会自动清)" : ""));
  else {
    const fresh = s.age < 1;
    setTile("#g-stage", GATE_STAGE[s.stage] || s.stage || "—",
            !fresh ? "bad" : s.published ? "ok" : "warn",
            [s.src, nz(s.dist) ? `距离 ${fx(s.dist, 2)} m` : "", s.pose && s.pose !== "ok" ? "位姿 " + s.pose : "",
             s.sim ? (verBad(s) ? "SIM (版本不符)" : "SIM") : ""].filter(Boolean).join(" · "));
  }
  // 控制段 (飞行脚本)
  const cmt = tg.commit || {};
  const past = cmt.passed || cmt.abort_ok === false;
  setTile("#g-seg", t ? (SEG[tg.seg] || tg.seg || PHASE[t.phase] || t.phase) : "脚本未运行",
          !t ? "" : past ? "warn" : isGatePhase(t.phase) ? "ok" : "",
          nz(cmt.d) ? `提交点 ${fx(cmt.d, 1)} m` + (past ? " · 已过, 不可中止" : " · 可中止") :
          t && !isGatePhase(t.phase) ? "还没进穿环段" : "");
  // 距离 / 偏差 / 离红点
  setTile("#g-along", nz(G && G.along) ? fx(G.along, 2, " m") : "—",
          nz(G && G.along) && G.along < 0 ? "ok" : "", G && G.along < 0 ? "已过环平面" : "沿法向，过环后为负");
  const off = G && G.off, aim = (G && G.aimOff) || [0, 0];
  const offBad = off && g && Math.hypot(off[0], off[1]) > g.r_in - 0.15;
  setTile("#g-off", off ? `${cm(off[0])}/${cm(off[1])} cm` : "—", offBad ? "bad" : "",
          g && g.n ? (g.aimIsCenter ? "红点按环中心（无 aim）" : `红点在 ${cm(aim[0])}/${cm(aim[1])} cm`) : "");
  const dd = g ? g.dot_d : GATE_DEF.dot_d;
  setTile("#g-miss", nz(G && G.miss) ? `${(G.miss * 100).toFixed(0)} cm` : "—",
          !nz(G && G.miss) ? "" : G.miss <= dd ? "ok" : G.miss <= (g ? g.r_in : 0.6) * 0.5 ? "warn" : "bad",
          G && G.pred ? "按脚本预计穿越点" : "按沿法向直飞");
  // 不确定度 / 速度 / 到环 / 感知年龄
  // σ: 横向 (靶面 h/v 的大者) 决定能不能穿环; 深度只影响停止点 —— 分开显示 (首飞问题 §1)
  const hva = tg.gate_used && Array.isArray(tg.gate_used.sd_hva_cm) && tg.gate_used.sd_hva_cm.every(nz)
    ? tg.gate_used.sd_hva_cm : null;
  const sd = hva ? Math.max(hva[0], hva[1]) / 100 : g && g.sd ? Math.max(g.sd[0], g.sd[1]) : null;
  const sdA = hva ? hva[2] / 100 : g && nz(g.sdAlong) ? g.sdAlong : null;
  setTile("#g-sd", nz(sd) ? `横 ${(sd * 100).toFixed(1)} cm` : "—", nz(sd) && sd > dd / 2 ? "warn" : nz(sd) ? "ok" : "",
          [nz(sdA) ? `深度 ${(sdA * 100).toFixed(1)} cm` : "", s && nz(s.n_sd_deg) ? `法向 ±${fx(s.n_sd_deg, 1)}°` : ""]
            .filter(Boolean).join(" · "));
  setTile("#g-v", nz(G && G.vAlong) ? fx(G.vAlong, 2, " m/s") : "—", "",
          [nz(G && G.vCmd) ? `指令 ${fx(G.vCmd, 2)}` : "", gateSpeedTxt(tg, gc)]
            .filter(Boolean).join(" · "));
  setTile("#g-ttc", nz(G && G.ttc) ? fx(G.ttc, 2, " s") : G && G.along < 0 ? "已过" : "—", "",
          nz(G && G.ttc) ? "按当前沿法向速度" : "");
  const gr = (S.rates || {}).gate || 0;
  setTile("#g-age", gateStale(s) ? "无新检测" : s && nz(s.age_ms) ? `${fx(s.age_ms, 0)} ms` : "—",
          !s || gateStale(s) ? "" : s.age > 1 ? "bad" : gr < 5 ? "warn" : "ok",
          s ? `${fx(gr, 1)} Hz` + (nz(s.ms) ? ` · 检测 ${fx(s.ms, 0)} ms` : "") : "");
  // 计数行
  const parts = [];
  if (s) {
    const r = s.ring && typeof s.ring === "object" ? s.ring : null;
    if (r) parts.push(`<span class="${r.ok ? "" : "warn"}">环 <b>${r.ok ? "✓" : "✗"}</b> ${esc(r.src || "")}` +
                      (r.corners ? ` 角点 <b>${r.corners}</b>` : "") + "</span>");
    parts.push(`<span>红点 <b>${Array.isArray(s.dots) && s.dots.length ? esc(s.dots.join("/")) : "—"}</b></span>`);
    parts.push(`<span class="${s.published ? "" : "warn"}">发布 <b>${s.published ? "是" : "否"}</b></span>`);
    parts.push(`<span class="${s.sim === false ? "warn" : "info"}">${s.sim === false ? "REAL (会驱动)" : "SIM (不驱动)"}</span>`);
    // ★2026-10-31 地面 / 飞机检测库、GateConfig 或消息版本不一致 → 桥强制 SIM
    if (verBad(s)) parts.push(`<span class="bad" title="${esc(s.ver_bad.join("\n"))}">` +
                              `版本不符 → 强制 SIM: <b>${esc(s.ver_bad.join("; "))}</b></span>`);
  }
  if (t && t.shadow && isGatePhase(t.phase)) parts.push('<span class="info">影子模式</span>');
  if (G && G.used && G.per && G.used.n && G.per.n) {
    const dc = Math.hypot(...V3.sub(G.used.c, G.per.c));
    parts.push(`<span class="${dc > 0.1 ? "warn" : ""}">锁定 vs 识别 <b>${(dc * 100).toFixed(0)}</b> cm</span>`);
  }
  $("#gate-counts").innerHTML = parts.join("");
  drawTarget($("#cv-tgt"), G);
}

function verBad(s) { return !!(s && Array.isArray(s.ver_bad) && s.ver_bad.length); }

// 靶面视图: 站在进近侧看向环 (右 = +h, 上 = +v), 原点 = 环中心
function drawTarget(cv, G) {
  const {ctx, w, h} = prep(cv);
  if (w < 30 || h < 30) return;
  ctx.font = "11px " + css("--mono");
  const g = G && G.geo;
  if (!g || !g.n) {
    ctx.fillStyle = C.dim; ctx.textAlign = "center";
    ctx.fillText(!g ? "等待环识别…" : "没有法向 n，画不了靶面", w / 2, h / 2);
    ctx.textAlign = "left"; return;
  }
  const aim = G.aimOff || [0, 0];
  let R = g.r_out * 1.2;
  for (const p of [aim, G.off, G.pred]) if (p) R = Math.max(R, Math.hypot(p[0], p[1]) + 0.12);
  R = Math.min(R, 2.5);
  const H0 = h - 14;
  const s = Math.min(w, H0) / 2 / R, cx = w / 2, cy = H0 / 2;
  const X = u => cx + u * s, Y = v => cy - v * s;
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1; ctx.beginPath();
  ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.moveTo(cx, 0); ctx.lineTo(cx, H0); ctx.stroke();
  // 环带
  ctx.beginPath(); ctx.arc(cx, cy, g.r_out * s, 0, Math.PI * 2); ctx.arc(cx, cy, g.r_in * s, 0, Math.PI * 2, true);
  ctx.fillStyle = C.tgt; ctx.globalAlpha = 0.28; ctx.fill("evenodd"); ctx.globalAlpha = 1;
  ctx.strokeStyle = C.tgt; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, g.r_out * s, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx, cy, g.r_in * s, 0, Math.PI * 2); ctx.stroke();
  const ell = (u, v, rx, ry, color, dash) => {
    if (!(rx * s > 1.5)) return;
    ctx.beginPath(); ctx.setLineDash(dash || []); ctx.strokeStyle = color; ctx.lineWidth = 1.2;
    ctx.ellipse(X(u), Y(v), rx * s, ry * s, 0, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
  };
  // 红点 (+ 环位置不确定度)
  ctx.fillStyle = "#e5534b"; ctx.beginPath();
  ctx.arc(X(aim[0]), Y(aim[1]), Math.max(3, g.dot_d / 2 * s), 0, Math.PI * 2); ctx.fill();
  if (g.aimIsCenter) { ctx.fillStyle = C.muted; ctx.fillText("红点按环中心", 4, H0 - 4); }
  if (g.sd) ell(aim[0], aim[1], g.sd[0], g.sd[1], C.tgt, [2, 3]);
  const inView = (u, v) => [clamp(X(u), 5, w - 5), clamp(Y(v), 5, H0 - 5)];
  // 预计穿越点 (脚本)
  if (G.pred) {
    const [px, py] = inView(G.pred[0], G.pred[1]);
    if (G.off) {
      const [ox, oy] = inView(G.off[0], G.off[1]);
      ctx.setLineDash([3, 3]); ctx.strokeStyle = C.sp; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(px, py); ctx.stroke(); ctx.setLineDash([]);
    }
    if (G.predSd) ell(G.pred[0], G.pred[1], G.predSd[0], G.predSd[1], C.sp);
    ctx.strokeStyle = C.sp; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(px - 5, py - 5); ctx.lineTo(px + 5, py + 5); ctx.moveTo(px + 5, py - 5); ctx.lineTo(px - 5, py + 5);
    ctx.stroke();
  }
  // 飞机 (沿法向投影到靶面)
  if (G.off) {
    const [ox, oy] = inView(G.off[0], G.off[1]);
    ctx.strokeStyle = C.text; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.arc(ox, oy, 6, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(ox - 9, oy); ctx.lineTo(ox + 9, oy); ctx.moveTo(ox, oy - 9); ctx.lineTo(ox, oy + 9);
    ctx.stroke();
  }
  ctx.fillStyle = C.muted; ctx.textAlign = "left";
  if (nz(G.along)) ctx.fillText(`距环 ${fx(G.along, 2)} m`, 4, 12);
  if (nz(G.miss)) {
    ctx.textAlign = "right";
    ctx.fillStyle = G.miss <= g.dot_d ? C.ok : G.miss <= g.r_in * 0.5 ? C.warn : C.bad;
    ctx.fillText(`离红点 ${(G.miss * 100).toFixed(0)} cm`, w - 4, 12);
  }
  ctx.textAlign = "left"; ctx.fillStyle = C.dim;
  ctx.fillText("○ 飞机  ✕ 预计穿越  ● 红点", 4, h - 2);
  ctx.textAlign = "right"; ctx.fillText(`环 Ø${(g.r_in * 200).toFixed(0)}/${(g.r_out * 200).toFixed(0)} cm`, w - 4, h - 2);
  ctx.textAlign = "left";
}

// ---------------- 通用曲线 ----------------
// o: {series:[{data,color,width}], band:[lo,hi], bandColor, bandNoScale, lines:[{v,color}], min, max, win, compact, minSpan}
function drawChart(cv, o) {
  if (!cv) return;
  const {ctx, w, h} = prep(cv);
  if (w < 10 || h < 10) return;
  const now = S ? S.now : 0, win = o.win || 30;
  const pad = o.compact ? {l: 1, r: 1, t: 3, b: 3} : {l: 44, r: 8, t: 6, b: 16};
  const series = o.series.map(s => Object.assign({}, s, {data: s.data.filter(p => now - p[0] <= win)}));
  let lo = o.min, hi = o.max;
  if (!nz(lo) || !nz(hi)) {
    const vals = [];
    for (const s of series) for (const p of s.data) vals.push(p[1]);
    if (o.band && nz(o.band[0]) && !o.bandNoScale) vals.push(o.band[0], o.band[1]);
    for (const l of o.lines || []) if (nz(l.v)) vals.push(l.v);
    if (!vals.length) vals.push(0, 1);
    let mn = Math.min(...vals), mx = Math.max(...vals);
    const sp = Math.max(mx - mn, o.minSpan || 0.1);
    if (!nz(lo)) lo = mn - sp * 0.1;
    if (!nz(hi)) hi = mx + sp * 0.1;
  }
  if (hi <= lo) hi = lo + 1;
  const X = t => pad.l + (1 - (now - t) / win) * (w - pad.l - pad.r);
  const Y = v => pad.t + (1 - (v - lo) / (hi - lo)) * (h - pad.t - pad.b);
  ctx.font = "10px " + css("--mono");
  if (!o.compact) {
    ctx.fillStyle = C.dim; ctx.textAlign = "right"; ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
    for (const v of [lo, (lo + hi) / 2, hi]) {
      ctx.beginPath(); ctx.moveTo(pad.l, Y(v)); ctx.lineTo(w - pad.r, Y(v)); ctx.stroke();
      ctx.fillText(v.toFixed(Math.abs(hi - lo) < 2 ? 2 : Math.abs(hi - lo) < 50 ? 1 : 0), pad.l - 4, Y(v) + 3);
    }
    ctx.textAlign = "left"; ctx.fillText(`-${win}s`, pad.l, h - 3);
    ctx.textAlign = "right"; ctx.fillText("现在", w - pad.r, h - 3);
  }
  if (o.band && nz(o.band[0])) {
    ctx.fillStyle = o.bandColor || C.band;
    const y1 = Y(clamp(o.band[1], lo, hi)), y0 = Y(clamp(o.band[0], lo, hi));
    ctx.fillRect(pad.l, y1, w - pad.l - pad.r, y0 - y1);
  }
  for (const l of o.lines || []) {
    if (!nz(l.v) || l.v < lo || l.v > hi) continue;
    ctx.strokeStyle = l.color || C.bad; ctx.lineWidth = 1; ctx.setLineDash([4, 3]); ctx.beginPath();
    ctx.moveTo(pad.l, Y(l.v)); ctx.lineTo(w - pad.r, Y(l.v)); ctx.stroke(); ctx.setLineDash([]);
  }
  let any = false;
  for (const s of series) {
    if (s.data.length < 2) continue;
    any = true;
    ctx.strokeStyle = s.color || C.text; ctx.lineWidth = s.width || (o.compact ? 1.4 : 1.6); ctx.beginPath();
    s.data.forEach((p, i) => { const x = X(p[0]), y = Y(clamp(p[1], lo, hi)); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
  }
  if (!any && !o.compact) { ctx.fillStyle = C.dim; ctx.textAlign = "center"; ctx.fillText("无数据", w / 2, h / 2); }
}
