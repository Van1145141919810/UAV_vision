# -*- coding: utf-8 -*-
"""monitor.html 里的内联 <script> 抽出来用 node --check 查语法 (还有 web/*.js, 有的话)。"""
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
bad = 0
files = []
for p in [os.path.join(ROOT, "monitor.html")] + [os.path.join(d, f) for d, _, fs in os.walk(os.path.join(ROOT, "vb09")) for f in fs if f.endswith((".html", ".js"))]:
    if not os.path.exists(p):
        continue
    s = open(p, encoding="utf-8").read()
    chunks = [s] if p.endswith(".js") else re.findall(r"<script(?![^>]*\bsrc=)[^>]*>(.*?)</script>", s, re.S)
    for i, js in enumerate(chunks):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
            f.write(js)
        r = subprocess.run(["node", "--check", f.name], capture_output=True, text=True)
        if r.returncode:
            bad += 1
            print("%s [script %d]:\n%s" % (os.path.relpath(p, ROOT), i, r.stderr[-800:]))
        files.append(p)
print("js syntax: %s (%d chunk(s))" % ("OK" if not bad else "%d bad" % bad, len(files)))
sys.exit(1 if bad else 0)
