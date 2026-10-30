// 页面冒烟: 起 flight_monitor --fake, 用 Chromium 打开每个标签页, 收集 console 错误 + 截图
//   node tools/dev/page_check.js <out_dir> [port]
const { chromium } = require('playwright');
const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');
const ROOT = path.resolve(__dirname, '..', '..');
const out = process.argv[2] || '/tmp/page_check';
const port = process.argv[3] || '8097';
fs.mkdirSync(out, { recursive: true });
(async () => {
  const home = fs.mkdtempSync('/tmp/pc_home_');
  const mon = spawn('python3', [path.join(ROOT, 'flight_monitor.py'), '--fake', '--fake-task', 'gate',
    '--port', port, '--nice', '0', '--sites-dir', path.join(ROOT, 'tools/baseline/sites')],
    { cwd: ROOT, env: Object.assign({}, process.env, { HOME: home }) });
  let log = '';
  mon.stdout.on('data', d => log += d); mon.stderr.on('data', d => log += d);
  await new Promise(r => setTimeout(r, 4000));
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1500, height: 1000 } });
  const errs = [];
  page.on('console', m => { if (m.type() === 'error' || m.type() === 'warning') errs.push(m.type() + ': ' + m.text()); });
  page.on('pageerror', e => errs.push('pageerror: ' + e.message));
  await page.goto('http://127.0.0.1:' + port + '/');
  await page.waitForTimeout(6000);
  await page.screenshot({ path: path.join(out, 'monitor.png'), fullPage: true });
  const tabs = await page.$$eval('nav a, .tabs a, [data-tab]', els => els.map(e => e.getAttribute('data-tab') || e.getAttribute('href') || e.textContent.trim()));
  fs.writeFileSync(path.join(out, 'tabs.txt'), tabs.join('\n'));
  let i = 0;
  for (const sel of await page.$$('nav a, .tabs a, [data-tab]')) {
    try { await sel.click(); await page.waitForTimeout(2500); await page.screenshot({ path: path.join(out, 'tab' + (i++) + '.png'), fullPage: true }); } catch (e) { errs.push('click: ' + e.message); }
  }
  // 再等一会让假飞机走完一圈 (冲刺段)
  await page.waitForTimeout(1000);
  fs.writeFileSync(path.join(out, 'console.txt'), errs.join('\n'));
  fs.writeFileSync(path.join(out, 'monitor.log'), log);
  console.log('tabs', tabs.length, 'console errors/warnings', errs.length);
  errs.slice(0, 20).forEach(e => console.log('  ' + e));
  await browser.close();
  mon.kill();
})();
