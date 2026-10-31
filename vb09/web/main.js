"use strict";
// vb09/web/main.js —— 尺寸变化 / 标签页 / 主题 / 启动
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 尺寸变化 → 重画 (画布尺寸由布局决定)
// =====================================================================
let rafPending = false;
function requestRender() {
  if (rafPending) return;
  rafPending = true;
  requestAnimationFrame(() => {
    rafPending = false;
    if (activeTab === "mon" && S) renderMonitor();
    else if (activeTab === "det" && S) renderDetail();
    else if (activeTab === "rep") renderReplay();
    else if (activeTab === "cfg" && LAST_PV) renderPreview(LAST_PV);
  });
}
if (window.ResizeObserver) {
  const ro = new ResizeObserver(requestRender);
  ro.observe(document.querySelector("main"));
  for (const el of $$(".cv")) ro.observe(el);
  ro.observe($("#cam-box"));
} else window.addEventListener("resize", requestRender);

// =====================================================================
// 标签页 / 主题 / 杂项
// =====================================================================
for (const b of $$(".tabs button")) b.addEventListener("click", () => {
  activeTab = b.dataset.tab;
  for (const x of $$(".tabs button")) x.classList.toggle("on", x === b);
  for (const s of $$(".tab")) s.classList.toggle("on", s.id === "tab-" + activeTab);
  if (activeTab === "cfg" && !SCHEMA) initConfig();
  if (activeTab === "cfg") pollLauncher(true).then(renderLauncher);
  if (activeTab === "help") initHelp();
  if (activeTab === "chk") loadPaths();
  if (activeTab === "rep") initReplay();
  requestRender();
});
for (const b of $$("#top-mode button")) {
  b.classList.toggle("on", b.dataset.v === topMode);
  b.addEventListener("click", () => {
    topMode = b.dataset.v; store("fm_topmode", topMode);
    for (const x of $$("#top-mode button")) x.classList.toggle("on", x === b);
    requestRender();
  });
}
for (const b of $$("#det-win button")) {
  b.classList.toggle("on", +b.dataset.v === detWin);
  b.addEventListener("click", () => {
    detWin = +b.dataset.v; store("fm_detwin", String(detWin));
    for (const x of $$("#det-win button")) x.classList.toggle("on", x === b);
    requestRender();
  });
}
$("#leg-btn").addEventListener("click", () => { $("#legend").hidden = !$("#legend").hidden; });
$("#legend").addEventListener("click", () => { $("#legend").hidden = true; });
function applyTheme(t) {
  if (t) document.documentElement.setAttribute("data-theme", t); else document.documentElement.removeAttribute("data-theme");
  loadColors(); requestRender();
}
applyTheme(store("fm_theme") || "");
$("#theme").addEventListener("click", () => {
  const cur = document.documentElement.getAttribute("data-theme") === "light" ? "" : "light";
  store("fm_theme", cur); applyTheme(cur);
});
$("#ev-info").addEventListener("change", e => $("#ev-list").classList.toggle("noinfo", !e.target.checked));
$("#cfg-load").addEventListener("click", loadSite);
$("#cfg-clear").addEventListener("click", () => { setKnobs({}); $("#cfg-sets").value = ""; schedulePreview(0); });
$("#cfg-sets").addEventListener("input", () => schedulePreview());
$("#sv-btn").addEventListener("click", () => save(false));
$("#sv-go").addEventListener("click", () => save(true));
$("#sv-temp").addEventListener("click", genTemp);
$("#sv-more").open = store("fm_sv_more") === "1";
$("#sv-more").addEventListener("toggle", () => store("fm_sv_more", $("#sv-more").open ? "1" : ""));
suggestName();
loadColors();
initFolds();
camInit();
pollState(); pollEvents(); pollChecks(); pollLauncher(); linkTick();
attLoop(); pollPaths(); pollStream();
restoreRef();
