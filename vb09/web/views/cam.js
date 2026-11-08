"use strict";
// vb09/web/views/cam.js —— 识别画面
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 识别画面: 从识别机的 preview_server 拉图
//   · 看完一张再要下一张 → 链路慢自动降速, 不和传图抢带宽
//   · 收起 / 切到别的标签页 / 浏览器标签不可见时完全不请求
//   · 用响应头 X-Frame-Age 判断画面是否卡住 (超过 1s 打红色遮罩)
//   · 打开时: 左栏 = 画面, 俯视图在右上; 关闭时: 俯视图搬到左栏放大
//   · 开关: 页头「识别画面」(快捷键 V) / 画面卡片的 ✕; 「数据」开关控制左下角检测数字
//   · 检测数字来自响应头 X-Det (preview_server 和画面同帧给出)
// =====================================================================
const CAM_DEFAULT = "http://127.0.0.1:8091/snapshot.jpg";
const CAM = {on: false, gen: 0, hist: [], det: null, detOn: true, proj: null, ovOn: true,
             main: store("fm_main") || "cam"};      // 画面打开时谁在左边大窗: "cam" | "top"
// 地址容错: 没写 http:// 就补上; 只写了 IP:端口 (或以 / 结尾) 就补 /snapshot.jpg
//   以 / 开头 = 本页同源路径 (比如以后由 flight_monitor 转发的 /cam/snapshot.jpg), 原样用
//   FAKE 模式下地址留空 → 用 flight_monitor 自带的假画面 /fake/snapshot.jpg
function camUrl() {
  let u = $("#cam-url").value.trim();
  if (!u) return S && S.mode === "fake" ? "/fake/snapshot.jpg" : CAM_DEFAULT;
  if (u.startsWith("/")) return u;
  if (!/^https?:\/\//i.test(u)) u = "http://" + u;
  try {
    const x = new URL(u);
    if (x.pathname === "/" || x.pathname === "") x.pathname = "/snapshot.jpg";
    return x.toString();
  } catch (e) { return u; }
}
function camOver(text, stale) {
  const o = $("#cam-over");
  if (!text) { o.hidden = true; return; }
  o.hidden = false; o.textContent = text; o.className = stale ? "stale" : "";
}
// 画面左下角的检测数字。m = null → 隐藏 (没开 / 连不上 / 识别程序是旧版没给 X-Det)
function renderDet(m) {
  CAM.det = m;
  // 这一帧的投影参数 (画面叠加用): {K:[fx,fy,cx,cy], size:[w,h], T_WC:[16 行主序], approx, t}
  const pj = m && m.proj;
  CAM.proj = pj && Array.isArray(pj.K) && pj.K.length === 4 && Array.isArray(pj.T_WC) && pj.T_WC.length >= 12
             && Array.isArray(pj.size) ? pj : null;
  drawCamOverlay();
  const el = $("#cam-det");
  if (!m || !CAM.detOn) { el.hidden = true; return; }
  // 环检测: gcs_gate.py 给 kind:"gate"; vision_bridge 的 /gate/status 原样转发时没有 kind, 但有 stage
  if (m.kind === "gate" || m.stage !== undefined) { renderGateDet(el, m); return; }
  const p = [m.found ? "✓ 检出" : "✗ " + (m.reason || "未检出")];
  if (nz(m.dist)) p.push(`相机距 ${m.dist.toFixed(2)} m`);
  if (nz(m.px)) p.push(`${m.px.toFixed(0)} px`);
  if (nz(m.rms)) p.push(`rms ${m.rms.toFixed(2)}`);
  if (Array.isArray(m.body) && m.body.every(nz))
    // 机体系 FLU: T_BC + R_BC·t_c, R_CAM_FWD 把相机 +z(光轴) 映射到机体 +x → 第 1 个分量是前向
    p.push(`机体(前${m.body[0].toFixed(2)} 左${m.body[1].toFixed(2)} 上${m.body[2].toFixed(2)})`);
  if (nz(m.rtt)) p.push(`往返 ${m.rtt.toFixed(0)} ms`);
  if (nz(m.stall) && m.stall > 0) p.push(`卡 ${m.stall}`);
  if (nz(m.det_ms)) p.push(`检测 ${m.det_ms.toFixed(0)} ms`);
  if (m.mode) p.push(m.probe ? m.mode + "·探针" : m.mode);
  el.hidden = false;
  el.textContent = p.join("  ·  ");
  el.title = el.textContent;
  el.style.color = m.found ? "#7ee787" : "#ffa198";
}
// ★2026-10-15 格子环 + 红点 (gcs_gate.py): 阶段/距离/来源/σ 是**飞机融合**的结果, 环/角点/红点/耗时是地面本帧检测
const GATE_STAGE = {search: "搜索", far: "远处·只有环", mid: "中距·角点平面", near: "近处·环出画",
  terminal: "末段·只看红点", coast: "惯性保持", blind: "盲区", lost: "丢失"};
// 法线来源 (gate_detector.pick_ring_normal): 前 4 种进估计器, mid 只显示, amb = 单目两解没分清
const GATE_NSRC = {plane: "角点平面", mono_plane: "单目·平面定向", mono_disp: "单目·视差定向",
  mono_est: "单目·估计器定向", mono_mid: "近正对(粗)", mono_amb: "二义?"};
function renderGateDet(el, m) {
  // ★2026-10-19 兼容 vision_bridge /gate/status 的原样格式: ring 是对象、det_ms 是字典、没有 found
  //   (以前直接 .toFixed 一个字典会抛异常, 整张画面变成"没有识别画面")
  const rg = m.ring && typeof m.ring === "object" ? m.ring : null;
  m = Object.assign({}, m);
  if (rg) { m.ring = rg.ok ? (rg.src || "✓") : ""; if (m.corners === undefined) m.corners = rg.corners; }
  if (m.det_ms && typeof m.det_ms === "object") m.det_ms = nz(m.det_ms.total) ? m.det_ms.total : m.ms;
  if (!nz(m.det_ms) || typeof m.det_ms !== "number") m.det_ms = typeof m.ms === "number" ? m.ms : null;
  if (m.found === undefined) m.found = !!(rg && rg.ok) || !!(Array.isArray(m.dots) && m.dots.length);
  const p = [];
  const stg = m.stage ? (GATE_STAGE[m.stage] || m.stage) : "—";
  p.push((m.published ? "✓ " : m.found ? "· " : "✗ ") + stg);
  if (nz(m.dist)) p.push(`距离 ${m.dist.toFixed(2)} m`);
  if (Array.isArray(m.sd_cm) && m.sd_cm.every(nz)) p.push(`σ ${Math.max(...m.sd_cm).toFixed(1)} cm`);
  if (m.src) p.push(m.src);
  if (m.ring) p.push(`环 ${m.ring}` + (m.corners ? ` ${m.corners}角点` : ""));
  if (m.dots && m.dots.length) p.push("红点 " + m.dots.join("/"));
  if (nz(m.z_ring)) p.push(`环距 ${m.z_ring.toFixed(1)} m`);
  if (nz(m.plane_rms)) p.push(`平面 ${(m.plane_rms * 1000).toFixed(0)} mm`);
  if (m.n_src) p.push(`法线 ${GATE_NSRC[m.n_src] || m.n_src}` + (nz(m.tilt) ? ` ${m.tilt.toFixed(0)}°` : ""));
  if (m.pose && m.pose !== "ok") p.push("位姿 " + m.pose);
  if (nz(m.det_ms)) p.push(`检测 ${m.det_ms.toFixed(0)} ms`);
  if (nz(m.rtt)) p.push(`往返 ${m.rtt.toFixed(0)} ms`);
  if (nz(m.stall) && m.stall > 0) p.push(`卡 ${m.stall}`);
  if (m.reason) p.push(m.reason);
  p.push(m.sim === false ? "REAL" : "SIM");
  el.hidden = false;
  el.textContent = p.join("  ·  ");
  el.title = el.textContent;
  el.style.color = m.published ? "#7ee787" : m.found ? "#e3b341" : "#ffa198";
}
// ★2026-10-19 画面叠加: 把 环 (紫 = 识别, 蓝 = 脚本锁定) / 红点 / 计划路径 / 预计穿越点 投影回画面。
//   投影参数来自这一帧的 X-Det.proj: K = 这张图 (size 尺寸) 的针孔内参, T_WC = 相机光学系
//   (x右 y下 z前) → world, 行主序 4×4。浏览器拿到的图可能被 ?w= 缩过, 按 naturalWidth / size 换算。
//   没有 proj (识别程序还没加) → 什么都不画。
function drawCamOverlay() {
  const cv = $("#cam-ov"), img = $("#cam-img");
  if (!cv) return;
  const {ctx, w, h} = prep(cv);
  if (CAM.on && w >= 10 && S) drawDotOverlay(ctx, w, h, img);     // 红点候选序号 / 点选框 (kinds/reddot.js)
  const P = CAM.proj;
  if (!CAM.on || !CAM.ovOn || !P || !img.naturalWidth || w < 10 || !S) return;
  const G = gateNow();
  if (!G.geo) return;
  const nw = img.naturalWidth, nh = img.naturalHeight;
  const sc = Math.min(w / nw, h / nh), ox = (w - nw * sc) / 2, oy = (h - nh * sc) / 2;
  const kx = nw / P.size[0], ky = nh / P.size[1];
  const T = P.T_WC.map(Number), K = P.K.map(Number), t = [T[3], T[7], T[11]];
  const proj = pw => {
    const d = V3.sub(pw, t);
    const xc = T[0] * d[0] + T[4] * d[1] + T[8] * d[2];
    const yc = T[1] * d[0] + T[5] * d[1] + T[9] * d[2];
    const zc = T[2] * d[0] + T[6] * d[1] + T[10] * d[2];
    if (zc < 0.05) return null;
    return [ox + (K[0] * xc / zc + K[2]) * kx * sc, oy + (K[1] * yc / zc + K[3]) * ky * sc];
  };
  const poly = (pts, color, lw, dash) => {
    ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.setLineDash(dash || []); ctx.beginPath();
    let pen = false;
    for (const p of pts) {
      const q = proj(p);
      if (!q) { pen = false; continue; }
      if (pen) ctx.lineTo(q[0], q[1]); else { ctx.moveTo(q[0], q[1]); pen = true; }
    }
    ctx.stroke(); ctx.setLineDash([]);
  };
  const circ = (g, c, r, n) => Array.from({length: n + 1}, (_, k) => {
    const a = 2 * Math.PI * k / n;
    return V3.add(c, V3.add(V3.mul(g.right, r * Math.cos(a)), V3.mul(g.up, r * Math.sin(a))));
  });
  ctx.font = "12px " + css("--mono");
  for (const [g, color] of [[G.per, C.tgt], [G.used, C.sp]]) {
    if (!g) continue;
    if (g.n) {
      poly(circ(g, g.c, g.r_out, 72), color, 2);
      poly(circ(g, g.c, g.r_in, 72), color, 1.5, [6, 4]);
      poly(circ(g, g.aim, Math.max(g.dot_d / 2, 0.03), 24), "#ff6b6b", 2);
    } else {
      const q = proj(g.c);
      if (q) {
        ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
        ctx.moveTo(q[0] - 10, q[1]); ctx.lineTo(q[0] + 10, q[1]); ctx.moveTo(q[0], q[1] - 10); ctx.lineTo(q[0], q[1] + 10);
        ctx.stroke();
      }
    }
  }
  const tg = G.tg || {};
  if (tg.plan && Array.isArray(tg.plan.pts)) {
    const pts = tg.plan.pts.map(vec3).filter(Boolean);
    poly(pts, C.sp, 2, [8, 5]);
    ctx.fillStyle = C.sp;
    for (const p of pts) {          // 计划点在画面里几乎排在一条视线上, 不写文字 (文字在俯视图里)
      const q = proj(p);
      if (q) ctx.fillRect(q[0] - 3, q[1] - 3, 6, 6);
    }
  }
  if (G.pred && G.geo.n) {
    const q = proj(planeToWorld(G.geo, G.pred[0], G.pred[1]));
    if (q) {
      ctx.strokeStyle = C.sp; ctx.lineWidth = 2.5; ctx.beginPath();
      ctx.moveTo(q[0] - 7, q[1] - 7); ctx.lineTo(q[0] + 7, q[1] + 7); ctx.moveTo(q[0] + 7, q[1] - 7); ctx.lineTo(q[0] - 7, q[1] + 7);
      ctx.stroke();
    }
  }
  ctx.textAlign = "right"; ctx.fillStyle = "rgba(255,255,255,.75)";
  ctx.fillText((P.approx ? "位姿近似 · " : "") + "叠加: 紫=识别 蓝=锁定/计划", w - 8, 18);
  ctx.textAlign = "left";
}
function applyCamLayout(on) {
  document.body.classList.toggle("cam-off", !on);
  const cam = $("#card-cam"), top = $("#card-top");
  cam.hidden = !on;
  $("#cam-sw").checked = on;
  if (on) {
    const big = CAM.main === "top" ? top : cam, small = big === top ? cam : top;
    $("#slot-main").prepend(big); $("#slot-side").prepend(small);
  } else $("#slot-main").appendChild(top);
  requestRender();
}
function camSwap() {
  CAM.main = CAM.main === "top" ? "cam" : "top";
  store("fm_main", CAM.main);
  applyCamLayout(CAM.on);
}
function camSet(on) {
  CAM.on = on; CAM.gen++; CAM.hist = [];
  store("fm_cam_on", on ? "1" : "");
  applyCamLayout(on);
  $("#cam-stat").textContent = on ? "连接中…" : "";
  renderDet(null);
  if (on) { camOver("连接中…"); camLoop(CAM.gen); }
  else { $("#cam-img").removeAttribute("src"); $("#cam-url-row").hidden = true; }
}
async function camLoop(gen) {
  if (gen !== CAM.gen || !CAM.on) return;                      // 已被新一轮取代或已收起
  let wait = +$("#cam-fps").value;
  // 看不见就不要图: 浏览器标签不在前台 / 不在监控页 / 画面卡片被收起
  if (document.hidden || activeTab !== "mon" || $("#card-cam").classList.contains("collapsed")) {
    setTimeout(() => camLoop(gen), 500); return;
  }
  const t0 = performance.now();
  try {
    const u = camUrl(), qv = $("#cam-q").value;
    const r = await fetch(u + (u.includes("?") ? "&" : "?") + (qv ? qv + "&" : "") + "n=" + Date.now(),
                          {cache: "no-store"});
    if (!r.ok) throw new Error(r.status !== 503 ? "HTTP " + r.status
      : qv === "src=raw" ? "识别程序没提供「原始」画面（检测程序是旧版？），换别的画质"
      : "识别程序还没交过画面（检测没在跑？mode 0？）");
    const ctype = r.headers.get("Content-Type") || "";
    if (!ctype.startsWith("image/"))
      throw new Error(`地址返回的不是图片 (${ctype || "无类型"}) —— 地址应以 /snapshot.jpg 结尾`);
    const age = parseFloat(r.headers.get("X-Frame-Age"));
    let det = null;
    try { det = JSON.parse(r.headers.get("X-Det") || "null"); } catch (e) { det = null; }
    const blob = await r.blob();
    const url = URL.createObjectURL(blob);
    const img = $("#cam-img");
    const loaded = await new Promise(res => {
      img.onload = () => res(true); img.onerror = () => res(false); img.src = url;
    });
    URL.revokeObjectURL(url);
    if (!loaded || !img.naturalWidth) throw new Error("收到的图片解不开 (%d KB)".replace("%d", (blob.size / 1024).toFixed(0)));
    if (gen !== CAM.gen) return;
    renderDet(det);
    const now = performance.now();
    CAM.hist.push([now, blob.size]);
    while (CAM.hist.length && now - CAM.hist[0][0] > 3000) CAM.hist.shift();
    const span = CAM.hist.length > 1 ? (now - CAM.hist[0][0]) / 1000 : 0;
    const fps = span > 0 ? (CAM.hist.length - 1) / span : 0;
    const kbps = span > 0 ? CAM.hist.slice(1).reduce((s, h) => s + h[1], 0) * 8 / 1000 / span : 0;
    // 分辨率 = 浏览器实际收到的; 上行 KB/张 = 飞机发来的原 JPEG 大小 (糊不糊的根源, 见帮助页)
    const up = det && nz(det.up_kb) ? ` · 上行 ${det.up_kb.toFixed(0)} KB/张` : "";
    $("#cam-stat").textContent = `帧龄 ${nz(age) ? (age * 1000).toFixed(0) + " ms" : "?"} · ` +
      `${img.naturalWidth}×${img.naturalHeight} · ${fps.toFixed(1)} 张/秒 · ` +
      `${(blob.size / 1024).toFixed(0)} KB · ≈${(kbps / 1000).toFixed(2)} Mbps${up}`;
    if (nz(age) && age > 1.0) camOver(`画面已停 ${age.toFixed(1)} s —— 识别程序卡住或不再 publish，别相信这张图`, true);
    else camOver("");
  } catch (e) {
    if (gen !== CAM.gen) return;
    renderDet(null);
    const u = camUrl(), m = String(e.message || e);
    const local = /\/\/(127\.0\.0\.1|localhost)[:/]/.test(u);
    let hint;
    if (e instanceof TypeError || /fetch|network/i.test(m)) {      // 连接被拒 / 地址不通 / 不是识别程序
      let hp = u; try { hp = new URL(u).host; } catch (e2) {}
      hint = local
        ? `${hp} 上没有识别程序的预览服务。\n① 浏览器和检测程序是不是同一台电脑？不是的话：检测程序加 --preview-host 0.0.0.0，` +
          "「地址」填 http://<检测机IP>:8091/snapshot.jpg\n② 检测程序在跑吗？终端里要有「[preview] http://…」那一行\n" +
          "③ 端口对吗？（默认 8091；检测程序启动时会打印该填的地址）"
        : `连不上 ${hp}\n① 检测程序加了 --preview-host 0.0.0.0 吗？\n② 检测机 Windows 防火墙放行 python 了吗？\n` +
          "③ IP 对吗？（检测程序启动时会打印该填的地址）";
    } else hint = m;
    camOver("没有识别画面：" + hint);
    $("#cam-stat").textContent = "连不上 · " + u;
    wait = 2000;
  }
  setTimeout(() => camLoop(gen), Math.max(0, wait - (performance.now() - t0)));
}
function camInit() {
  $("#cam-url").value = store("fm_cam_url") || "";
  const f = store("fm_cam_fps"); if (f) $("#cam-fps").value = f;
  // 显示 / 关闭画面: 页头开关、卡片 ✕、快捷键 V (在输入框里打字时不触发)
  $("#cam-sw").addEventListener("change", e => camSet(e.target.checked));
  $("#cam-off").addEventListener("click", () => camSet(false));
  document.addEventListener("keydown", e => {
    if (e.ctrlKey || e.metaKey || e.altKey || e.repeat) return;
    // 只在打字的地方屏蔽 (文本框 / 下拉框); 刚点过的开关 (checkbox) 不算
    const el = e.target;
    if (el && el.matches && el.matches(
        "textarea, select, [contenteditable], input:not([type=checkbox]):not([type=radio])")) return;
    if (e.key === "v" || e.key === "V") { e.preventDefault(); camSet(!CAM.on); }
  });
  // 叠加的检测数字
  CAM.detOn = store("fm_cam_det") !== "0";
  $("#det-sw").checked = CAM.detOn;
  $("#det-sw").addEventListener("change", e => {
    CAM.detOn = e.target.checked; store("fm_cam_det", CAM.detOn ? "1" : "0"); renderDet(CAM.det);
  });
  // 画面叠加 (环 / 红点 / 计划路径)
  CAM.ovOn = store("fm_cam_ov") !== "0";
  $("#ov-sw").checked = CAM.ovOn;
  $("#ov-sw").addEventListener("change", e => {
    CAM.ovOn = e.target.checked; store("fm_cam_ov", CAM.ovOn ? "1" : "0"); drawCamOverlay();
  });
  $("#cam-cfg").addEventListener("click", () => { const r = $("#cam-url-row"); r.hidden = !r.hidden; if (!r.hidden) $("#cam-url").focus(); });
  const apply = () => { store("fm_cam_url", $("#cam-url").value.trim()); $("#cam-url-row").hidden = true; if (CAM.on) camSet(true); };
  $("#cam-url-ok").addEventListener("click", apply);
  $("#cam-url").addEventListener("keydown", e => { if (e.key === "Enter") apply(); if (e.key === "Escape") $("#cam-url-row").hidden = true; });
  $("#cam-fps").addEventListener("change", () => store("fm_cam_fps", $("#cam-fps").value));
  // 画质 (记住选择; 换画质时清掉带宽统计)
  const q = store("fm_cam_q");
  if (q !== null && [...$("#cam-q").options].some(o => o.value === q)) $("#cam-q").value = q;
  $("#cam-q").addEventListener("change", () => { store("fm_cam_q", $("#cam-q").value); CAM.hist = []; });
  // 全屏: ⛶ 或双击画面; Esc 退出
  const fs = () => {
    if (document.fullscreenElement) document.exitFullscreen();
    else if ($("#cam-box").requestFullscreen) $("#cam-box").requestFullscreen().catch(() => {});
  };
  $("#cam-fs").addEventListener("click", fs);
  $("#cam-box").addEventListener("dblclick", e => { if (!e.target.closest("#cam-url-row")) fs(); });
  document.addEventListener("fullscreenchange", requestRender);
  // 主次互换
  $("#cam-swap").addEventListener("click", camSwap);
  $("#top-swap").addEventListener("click", camSwap);
  camSet(store("fm_cam_on") === "1");
  dotInit();                       // 画面点选 (kinds/reddot.js)
}
