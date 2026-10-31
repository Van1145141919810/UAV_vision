"use strict";
// vb09/web/kinds/ring.js —— 穿环: 几何 + 数据来源
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// ★2026-10-19 穿环: 几何 + 数据来源   (字段约定: 同目录 INTERFACE_gate.md)
//   world = VIO odom (ENU), 和 /mavros/local_position/pose 同一个系
//   n     = 环平面法向, 指向**进近侧** (飞机过来的那一边)
//   靶面坐标: 从进近侧看向环, h = 右为正, v = 上为正; along = (p − c)·n, 穿过环平面时 0, 过环后为负
//   两个来源, 网页里一直分开标:
//     感知  vision_bridge 的 /gate/status —— 什么时候都有, 不需要飞行脚本
//     控制  飞行脚本遥测里的 gate 段 —— 脚本实际在用 (锁定) 的环、它算的偏差/预计穿越点
// =====================================================================
const V3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  mul: (a, k) => [a[0] * k, a[1] * k, a[2] * k],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  unit: a => { const n = Math.hypot(a[0], a[1], a[2]); return n > 1e-9 ? [a[0] / n, a[1] / n, a[2] / n] : null; },
};
const vec3 = a => Array.isArray(a) && a.length >= 3 && nz(a[0]) && nz(a[1]) && nz(a[2]) ? [+a[0], +a[1], +a[2]] : null;
// 配置里没有 derived.gate 时的兜底尺寸 (环内径 1.2 m / 外径 1.6 m / 红点直径 11 cm)
const GATE_DEF = {r_in: 0.6, r_out: 0.8, dot_d: 0.11};
function gateCfg() { return (D() || {}).gate || null; }
function gateDims(src) {
  const g = gateCfg() || {};
  const pick = k => nz(src && src[k]) ? +src[k] : nz(g[k]) ? +g[k] : GATE_DEF[k];
  return {r_in: pick("r_in"), r_out: pick("r_out"), dot_d: pick("dot_d")};
}
function gateAxes(n) {
  const d = V3.mul(n, -1);
  const right = V3.unit(V3.cross(d, [0, 0, 1])) || [0, -1, 0];
  return {right, up: V3.cross(right, d)};
}
// 环对象 {kind, c, n, right, up, aim, aimIsCenter, sd:[h,v] m, r_in, r_out, dot_d, ...}; 没有 n 时只有中心点
function mkGate(kind, c, n, aim, sdcm, cov, extra) {
  if (!c) return null;
  const g = Object.assign({kind, c, n: n ? V3.unit(n) : null}, extra || {});
  Object.assign(g, gateDims(extra));
  g.aimIsCenter = !aim; g.aim = aim || c;
  if (g.n) Object.assign(g, gateAxes(g.n));
  let sh = null, sv = null;     // 靶面内的位置不确定度: 有协方差就投到 right / up, 否则取各轴 σ 最大的
  if (g.n && Array.isArray(cov) && cov.length === 9 && cov.every(nz)) {
    const q = u => Math.sqrt(Math.max(0,
      u[0] * (cov[0] * u[0] + cov[1] * u[1] + cov[2] * u[2]) +
      u[1] * (cov[3] * u[0] + cov[4] * u[1] + cov[5] * u[2]) +
      u[2] * (cov[6] * u[0] + cov[7] * u[1] + cov[8] * u[2])));
    sh = q(g.right); sv = q(g.up); g.sdAlong = q(g.n);
  } else if (Array.isArray(sdcm) && sdcm.length && sdcm.every(nz)) sh = sv = Math.max(...sdcm) / 100;
  g.sd = nz(sh) ? [sh, sv] : null;
  return g;
}
// ★2026-10-19 (穿环首飞问题 §2): 桥里的环估计 age_ms 超过它 = "无新检测", 不再拿旧值画图/算数
const GATE_STALE_MS = 500;
const gateStale = s => !!s && nz(s.age_ms) && s.age_ms > GATE_STALE_MS;
function gatePerceived() {
  const s = S && S.gate;
  if (!s || !(s.age < 3) || gateStale(s)) return null;
  // 新版桥: target = 红点的融合估计, ring_c = 环心 (2026-10-19); 老版桥没有 ring_c, target 当环心
  return mkGate("感知", vec3(s.ring_w || s.ring_c || s.target), vec3(s.n), vec3(s.aim || (s.ring_c ? s.target : null)),
                s.ring_w ? null : s.sd_cm, s.ring_w ? s.ring_cov : s.cov,
                {stage: s.stage, published: !!s.published, r_in: s.r_in, r_out: s.r_out});
}
function gateUsed() {
  const t = tel(), u = t && t.gate && t.gate.gate_used;
  if (!u) return null;
  return mkGate("控制", vec3(u.c), vec3(u.n), vec3(u.aim), u.sd_cm, u.cov,
                {locked: !!u.locked, r_in: u.r_in, r_out: u.r_out, n_src: u.n_src});
}
function toPlane(g, p) {
  const q = V3.sub(p, g.c);
  return {along: V3.dot(q, g.n), h: V3.dot(q, g.right), v: V3.dot(q, g.up)};
}
function planeToWorld(g, h, v) { return V3.add(g.c, V3.add(V3.mul(g.right, h), V3.mul(g.up, v))); }
function dronePos() { return S && S.pose && S.pose.age < 2 ? [S.pose.x, S.pose.y, S.pose.z] : null; }
// 当前任务。脚本在跑 → 它说了算; 否则配置里的 task; 都没有 = 穿环 (2026-10-30 起只有穿环)
function visionMode() { const c = S && S.cam; return c && c.age < 5 && c.cfg ? c.cfg.mode : null; }
function curTask() {
  const t = tel();
  if (t) return t.task || "gate";
  const cs = cfgSrc();
  return (cs && cs.c.task) || "gate";
}
// 监控页用穿环布局。2026-10-30 停靠跟踪布局已删除 → 总是 true (保留函数, 以后按目标类型切换布局时用)
function gateView() { return true; }
// 穿环卡片 / 靶面 / 俯视图共用的一组数
function gateNow() {
  const t = tel(), tg = t && t.gate && Object.keys(t.gate).length ? t.gate : null;
  const per = gatePerceived(), used = gateUsed();
  const geo = (used && used.n) ? used : (per && per.n) ? per : (used || per);
  const o = {per, used, geo, tg, seg: tg ? tg.seg : null, src: null};
  if (!geo) return o;
  if (geo.n) { const q = toPlane(geo, geo.aim); o.aimOff = [q.h, q.v]; } else o.aimOff = [0, 0];
  if (tg && nz(tg.along)) {
    o.src = "控制";
    o.along = tg.along;
    o.off = Array.isArray(tg.off) && tg.off.every(nz) ? tg.off : null;
    o.pred = Array.isArray(tg.pred_cross) && tg.pred_cross.every(nz) ? tg.pred_cross : null;
    o.predSd = Array.isArray(tg.pred_sd_cm) && tg.pred_sd_cm.every(nz) ? tg.pred_sd_cm.map(v => v / 100) : null;
    o.ttc = tg.ttc; o.vAlong = tg.v_along_meas; o.vCmd = tg.v_along_cmd;
  } else if (geo.n) {
    const p = dronePos();
    if (p) {
      o.src = "感知";
      const q = toPlane(geo, p);
      o.along = q.along; o.off = [q.h, q.v];
      if (S.vel && S.vel.age < 1) {
        o.vAlong = -V3.dot([S.vel.x, S.vel.y, S.vel.z], geo.n);
        if (o.vAlong > 0.2 && o.along > 0) o.ttc = o.along / o.vAlong;
      }
    }
  }
  const ref = o.pred || o.off;          // 离红点: 有预计穿越点用它, 否则用"沿法向直飞会穿过的点"
  if (ref) o.miss = Math.hypot(ref[0] - o.aimOff[0], ref[1] - o.aimOff[1]);
  return o;
}
