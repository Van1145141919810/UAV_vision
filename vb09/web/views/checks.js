"use strict";
// vb09/web/views/checks.js —— 机况检查
// 2026-10-30 第 5 步从 monitor.html 的内联脚本原样切出 (按原顺序用普通 <script> 加载, 共用全局作用域)
// =====================================================================
// 机况检查
// =====================================================================
async function pollChecks() {
  if (activeTab === "chk") {
    try {
      const r = await getJSON("/api/checks");
      const tb = $("#chk-body");
      const label = {ok: "正常", warn: "注意", fail: "不通过", info: "信息"};
      tb.innerHTML = r.checks.map(c => `<tr><td><span class="st-p ${c.status}">${label[c.status] || c.status}</span></td>
        <td>${esc(c.name)}</td><td class="num">${esc(c.value)}</td><td class="muted small">${esc(c.detail)}</td></tr>`).join("");
      const nf = r.checks.filter(c => c.status === "fail").length, nw = r.checks.filter(c => c.status === "warn").length;
      const sum = $("#chk-sum");
      sum.textContent = nf ? `${nf} 项不通过 · ${nw} 项注意` : nw ? `${nw} 项注意` : "全部正常";
      sum.className = nf ? "bad" : nw ? "warn" : "ok";
    } catch (e) {}
  }
  setTimeout(pollChecks, 1000);
}
