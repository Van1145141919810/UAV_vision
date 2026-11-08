"use strict";
// vb09/web/kinds/reddot.js —— 红点 (跟踪类目标) 的网页部分 (2026-11-08, claude/PLAN_dot_v2.md)
// =====================================================================
// · 穿环卡片下方的「红点」区: 选点状态、候选列表 (每个一个「选」按钮)、抵近 / 换视角状态、立体比例、法向
// · 识别画面点选: 按「画面点选」后点画面上的红点 → 那一帧的像素 + 拍摄时刻 → 飞行脚本 (找目标时生效) → 桥按视线挂轨迹
// 数据: S.gate = /gate/status (桥里 Locator.extra: kind=reddot, sel, cands, n_state, st_scale …);
//       tel().gate.track = 飞行脚本 steps/track.py 的状态; CAM.det.pick = 地面 gcs_gate 给的这张图的内参 + t_cap
// 选点请求: POST /api/select → flight_monitor → /flight_ctrl/select → 飞行脚本 → /gate/select → 桥
// 普通 <script>, 和其它文件共享全局作用域; 在 cam.js 之前加载 (drawCamOverlay 调 drawDotOverlay)
// =====================================================================
const DOT = {pick: false, msg: "", msgT: 0, html: ""};
const DOT_SEL = {none: "没选", pending: "等视线上出现候选", bound: "已选", lost: "选中的丢了 (按位置重新找)"};
const DOT_STEP = {detect: "找目标 / 选点", approach: "抵近 (边飞边估)", refine: "判够不够准", locked: "已锁定"};
const DOT_NST = {ok: "已定", frontal: "近正对 (按视线)", ambig: "两个解分不清", span: "视角跨度不够", few: "视角太少"};

function dotActive() { return !!(S && S.gate && S.gate.kind === "reddot"); }

async function dotPost(cmd) {
  try {
    const r = await getJSON("/api/select", {method: "POST", headers: {"Content-Type": "application/json"},
                                            body: JSON.stringify(cmd)});
    DOT.msg = r.ok ? `已发: ${cmd.mode === "track" ? "轨迹 " + cmd.trk_id : cmd.mode === "ray" ? "画面点选" : "取消选点"}` +
                     " —— 手动段立即生效, 否则找目标 (wait 段) 时生效"
                   : "没发出去: " + (r.error || "?");
  } catch (e) { DOT.msg = "没发出去: " + e; }
  DOT.msgT = Date.now();
  renderDot(tel());
}

function renderDot(t) {
  const box = $("#dot-box");
  if (!box) return;
  if (!dotActive()) { box.hidden = true; DOT.html = ""; return; }
  box.hidden = false;
  const s = S.gate, tg = (t && t.gate) || {}, tk = tg.track || {};
  const sel = s.sel || {};
  const cands = Array.isArray(s.cands) ? s.cands : [];
  const rows = cands.map((c, i) => {
    const on = sel.state === "bound" && sel.trk_id === c.id;
    const D = nz(c.D_trk) ? fx(c.D_trk * 100, 1) : nz(c.D) ? "~" + fx(c.D * 100, 0) : "—";
    return `<tr class="${on ? "on" : ""}"><td>${i + 1}</td><td>${c.id}</td><td>${fx(c.score, 2)}</td><td>${c.hits ?? "—"}</td>` +
           `<td>${D}</td><td>${fx(c.Z_st, 2)}</td><td>${fx(c.sd_cm, 1)}</td><td>${fx(c.age, 1)}</td>` +
           `<td><button class="btn sm" data-trk="${c.id}" ${on ? "disabled" : ""}>${on ? "已选" : "选"}</button></td></tr>`;
  }).join("");
  const sg = Array.isArray(tk.sig) ? `σ 横 ${fx(tk.sig[0], 1)} / 竖 ${fx(tk.sig[1], 1)} / 深 ${fx(tk.sig[2], 1)} cm` :
             Array.isArray(s.aim_sd_cm) ? `σ ${s.aim_sd_cm.map(v => fx(v, 1)).join(" / ")} cm (xyz)` : "";
  const parts = [
    `选点: <b>${DOT_SEL[sel.state] || sel.state || "—"}</b>` + (nz(sel.trk_id) ? ` (轨迹 ${sel.trk_id})` : ""),
    tk.step ? `步骤: <b>${DOT_STEP[tk.step] || tk.step}</b>` + (nz(tk.rounds) && tk.rounds > 0 ? ` · 换视角 ${tk.rounds} 轮` : "") : "",
    tk.step === "approach" && nz(tk.dist_cmd) ? `停在 ${fx(tk.dist_cmd, 2)} m (方向按${tk.dir_src || "?"})` : "",
    sg,
    s.n_state ? `法向: ${DOT_NST[s.n_state] || s.n_state}` + (nz(s.n_sd_deg) && s.n ? ` ±${fx(s.n_sd_deg, 1)}°` : "") : "",
    nz(s.st_scale) ? `立体比例 ${fx(s.st_scale, 3)} ±${fx(s.st_scale_sd, 3)}` : "",
    nz(tk.relock) ? `重锁过 (差 ${fx(tk.relock, 1)} cm)` : "",
  ].filter(Boolean);
  const why = tk.why ? `<div class="dot-why">${esc(tk.why)}</div>` : "";
  const msg = DOT.msg && Date.now() - DOT.msgT < 8000 ? `<span class="dot-msg">${esc(DOT.msg)}</span>` : "";
  const html =
    `<div class="dot-h"><b>红点</b><span class="small muted">${parts.join("  ·  ")}</span>` +
    `<span class="r"><button class="btn sm ${DOT.pick ? "on" : ""}" id="dot-pick" ` +
    `title="按下后点识别画面上的红点 (远处置信度低、自动选不出来时用)">${DOT.pick ? "点画面…(Esc 取消)" : "画面点选"}</button>` +
    `<button class="btn sm" id="dot-none" title="取消选点: 回到发布分数最高的">取消选点</button></span></div>` + why + msg +
    (rows ? `<table class="dot-tbl"><tr><th>#</th><th>轨迹</th><th>分数</th><th>命中</th><th>直径 cm</th><th>Z m</th>` +
            `<th>σ cm</th><th>旧 s</th><th></th></tr>${rows}</table>`
          : `<div class="small muted">没有候选</div>`);
  if (html === DOT.html) return;             // 每帧都会调: 内容没变就不重建 (否则按钮在点下去的瞬间被换掉)
  DOT.html = html;
  box.innerHTML = html;
  box.querySelectorAll("button[data-trk]").forEach(b =>
    b.addEventListener("click", () => dotPost({mode: "track", trk_id: +b.dataset.trk})));
  $("#dot-pick").addEventListener("click", () => {
    DOT.pick = !DOT.pick;
    if (DOT.pick && !CAM.on) { DOT.msg = "先打开识别画面 (V)"; DOT.msgT = Date.now(); DOT.pick = false; }
    renderDot(tel()); drawCamOverlay();
  });
  $("#dot-none").addEventListener("click", () => dotPost({mode: "none"}));
}

// 画面点选: 鼠标位置 → 这张图的像素 → 校正左目归一化坐标 (内参来自 X-Det.pick)
function dotPickAt(e) {
  if (!DOT.pick) return;
  const img = $("#cam-img"), m = CAM.det, pk = m && m.pick;
  DOT.pick = false;
  if (!pk || !img.naturalWidth) { DOT.msg = "这张图没有内参 (地面 gcs_gate 是旧版? 或者不是红点检测)"; }
  else if ($("#cam-q").value === "src=raw") { DOT.msg = "「原始」画质是没校正的并排图, 不能点选 —— 换别的画质"; }
  else {
    const r = $("#cam-box").getBoundingClientRect();
    const nw = img.naturalWidth, nh = img.naturalHeight;
    const sc = Math.min(r.width / nw, r.height / nh), ox = (r.width - nw * sc) / 2, oy = (r.height - nh * sc) / 2;
    const un = (e.clientX - r.left - ox) / sc, vn = (e.clientY - r.top - oy) / sc;
    const u = un * pk.img_w / nw, v = vn * pk.h / nh;
    if (un < 0 || vn < 0 || un > nw || vn > nh) DOT.msg = "点到画面外了";
    else if (u > pk.w) DOT.msg = "点到右边的掩膜图了, 点左边的画面";
    else { dotPost({mode: "ray", xn: (u - pk.cx) / pk.f, yn: (v - pk.cy) / pk.f, t_cap: pk.t_cap}); return; }
  }
  DOT.msgT = Date.now();
  renderDot(tel()); drawCamOverlay();
}

// 画面叠加 (cam.js drawCamOverlay 调用): 点选模式下给每个候选标上序号 (和表格的 # 一致)
function drawDotOverlay(ctx, w, h, img) {
  if (!dotActive() || !CAM.det || !img.naturalWidth) return;
  const pk = CAM.det.pick, cs = CAM.det.dot_cands;
  if (!pk || !Array.isArray(cs)) return;
  const nw = img.naturalWidth, nh = img.naturalHeight;
  const sc = Math.min(w / nw, h / nh), ox = (w - nw * sc) / 2, oy = (h - nh * sc) / 2;
  const sel = (S.gate && S.gate.sel) || {};
  ctx.font = "bold 14px " + css("--mono");
  cs.forEach((c, i) => {
    if (!Array.isArray(c.uv)) return;
    const x = ox + c.uv[0] * nw / pk.img_w * sc, y = oy + c.uv[1] * nh / pk.h * sc;
    const on = sel.state === "bound" && sel.trk_id === c.id;
    ctx.fillStyle = on ? "#7ee787" : "#e3b341";
    ctx.fillText(String(i + 1), x + 10, y - 10);
  });
  if (DOT.pick) {
    ctx.strokeStyle = "#e3b341"; ctx.lineWidth = 3; ctx.strokeRect(2, 2, w - 4, h - 4);
  }
}

function dotInit() {
  const box = $("#cam-box");
  if (box) box.addEventListener("click", dotPickAt);
  document.addEventListener("keydown", e => {
    if (e.key === "Escape" && DOT.pick) { DOT.pick = false; renderDot(tel()); drawCamOverlay(); }
  });
}
