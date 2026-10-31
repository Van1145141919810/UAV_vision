"use strict";
// vb09/web/ui.js —— 卡片收起 + 文件与路径
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 卡片收起: 每张卡片标题栏右边一个 ▾ (点标题文字也行), 状态按卡片记在浏览器里
//   收起后只剩标题栏, 同一列的其他卡片 (俯视图 / 事件) 自动长大
// =====================================================================
function initFolds() {
  const seen = {};
  for (const card of $$(".card")) {
    const h = card.querySelector(":scope > .card-h");
    if (!h || h.querySelector(".fold")) continue;
    const tab = (card.closest(".tab") || {}).id || "x";
    seen[tab] = (seen[tab] || 0) + 1;
    const key = "fm_fold_" + (card.id || tab + "_" + seen[tab]);
    const b = document.createElement("button");
    b.type = "button"; b.className = "fold"; b.title = "收起 / 展开";
    if (!h.querySelector(":scope > .r")) b.style.marginLeft = "auto";
    h.appendChild(b);
    const set = (c, save) => {
      card.classList.toggle("collapsed", c);
      b.textContent = c ? "▸" : "▾";
      b.setAttribute("aria-expanded", String(!c));
      if (save) store(key, c ? "1" : "");
      requestRender();
    };
    const flip = e => { e.stopPropagation(); set(!card.classList.contains("collapsed"), true); };
    b.addEventListener("click", flip);
    const t = h.querySelector("h3"); if (t) t.addEventListener("click", flip);
    set(store(key) === "1", false);
  }
}

// =====================================================================
// 机况检查 → 文件与路径
// =====================================================================
async function pollPaths() {
  if (activeTab === "chk") await loadPaths();
  setTimeout(pollPaths, 5000);
}
async function loadPaths() {
  {
    try {
      const r = await getJSON("/api/paths");
      $("#paths-host").textContent = "飞机 " + (r.host || "?");
      const st = x => x === null ? '<span class="st-p info">—</span>'
                    : x ? '<span class="st-p ok">在</span>' : '<span class="st-p fail">不在</span>';
      $("#paths-body").innerHTML = r.rows.map(x => `<tr><td>${st(x.exists)}</td><td>${esc(x.name)}</td>
        <td class="num small" style="word-break:break-all">${esc(x.path)}${x.real ? ` <span class="muted">→ ${esc(x.real)}</span>` : ""}</td>
        <td class="muted small">${esc(x.note)}</td></tr>`).join("");
    } catch (e) {}
  }
}
