"use strict";
// vb09/web/views/help.js —— 帮助页
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 帮助页: 命令随输入框变化; 飞行脚本名 / 目录 / 档案列表来自飞机
// =====================================================================
let HELP = null;           // /api/sites 的结果
async function initHelp() {
  const gcs = $("#hp-gcs"), ip = $("#hp-ip");
  if (!gcs.dataset.init) {
    gcs.dataset.init = "1";
    gcs.value = store("fm_gcs_ip") || "192.168.2.179";
    ip.value = store("fm_uav_ip") || "192.168.2.20";
    gcs.addEventListener("input", () => { store("fm_gcs_ip", gcs.value.trim()); renderHelp(); });
    ip.addEventListener("input", () => { store("fm_uav_ip", ip.value.trim()); renderHelp(); });
    $("#hp-site").addEventListener("change", renderHelp);
  }
  if (!HELP) {
    try { HELP = await getJSON("/api/sites"); } catch (e) { HELP = {}; }
    const cur = $("#hp-site").value || baseName;
    $("#hp-site").innerHTML = `<option value="">（不用档案 = 代码默认值）</option>` +
      (HELP.sites || []).map(s => `<option value="${esc(s.name)}">${esc(s.name)}</option>`).join("");
    if (cur) $("#hp-site").value = cur;
  }
  renderHelp();
}
function renderHelp() {
  const gcs = $("#hp-gcs").value.trim() || "192.168.2.179";
  const ip = $("#hp-ip").value.trim() || "192.168.2.20";
  const okIp = s => /^\d{1,3}(\.\d{1,3}){3}$/.test(s);
  $("#hp-gcs").style.borderColor = okIp(gcs) ? "" : "var(--bad)";
  $("#hp-ip").style.borderColor = okIp(ip) ? "" : "var(--bad)";
  const h = HELP || {};
  // ★2026-10-19 按当前任务选脚本和视觉模式
  const task = curTask();
  const run = (h.scripts && h.scripts[task] && h.scripts[task].cmd) || h.script_cmd ||
              "cd ~/<目录> && python3 stageNN_" + task + ".py";
  const site = $("#hp-site").value;
  const siteArg = site ? ` --site ${site}` : "";
  const monDir = (h.monitor_dir || "~/<目录>").replace(/^\/home\/[^/]+/, "~").replace(/^\/root/, "~");
  const row = (id, cmd) => {
    $(id).innerHTML = `<div class="cmd"><code>${esc(cmd)}</code><button class="btn" data-copy="${esc(cmd)}">复制</button></div>`;
  };
  row("#hp-c-mavros", `roslaunch mavros px4_visquad.launch fcu_url:=/dev/ttyS7:921600 gcs_url:=udp://@${gcs}:14550`);
  row("#hp-c-vins", "roslaunch vins stereo120.launch");
  row("#hp-c-kill", "rosnode kill /geometric_controller");
  row("#hp-c-mon", `cd ${monDir} && nice -n 10 python3 flight_monitor.py`);
  row("#hp-c-tunnel", `ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -L 8080:127.0.0.1:8080 visbot@${ip}`);
  row("#hp-c-bridge", `cd ${monDir} && python3 run_vision.py ${task === "gate" ? "gate_gcs" : "aruco"} --gcs ${gcs}`);
  row("#hp-c-ssh", `ssh visbot@${ip}`);
  row("#hp-c-dry", `${run}${siteArg} --dry-run`);
  row("#hp-c-fly", `${run}${siteArg}`);
  row("#hp-c-fly2", h.launcher || "~/fly.sh");
}
