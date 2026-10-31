# -*- coding: utf-8 -*-
"""flight_monitor --fake 的 API 快照 (重构前后对比用): python3 api_snapshot.py <root> <port> <out.json>"""
import json, os, re, subprocess, sys, tempfile, time, urllib.request
root, port, out = sys.argv[1], sys.argv[2], sys.argv[3]
home = tempfile.mkdtemp()
sites = os.path.join(home, "flight_sites")
os.makedirs(sites)
for f in ("sim.conf", "adv.conf", "shadow.conf"):
    open(os.path.join(sites, f), "w").write(open(os.path.join(os.path.dirname(__file__), "..", "baseline", "sites", f)).read())
p = subprocess.Popen([sys.executable, os.path.join(root, "flight_monitor.py"), "--fake", "--port", port, "--nice", "0",
                      "--sites-dir", sites, "--launcher", os.path.join(home, "fly.sh")],
                     cwd=root, env=dict(os.environ, HOME=home), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(4)
base = "http://127.0.0.1:%s" % port
def get(path):
    return json.loads(urllib.request.urlopen(base + path, timeout=30).read())
def post(path, body):
    r = urllib.request.Request(base + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=60).read())
res = {}
try:
    res["schema"] = get("/api/schema")
    res["sites"] = get("/api/sites")
    res["site_sim"] = get("/api/site?name=sim")
    res["preview_adv"] = post("/api/preview", {"knobs": get("/api/site?name=adv").get("knobs"), "sets": []})
    res["preview_shadow"] = post("/api/preview", {"knobs": get("/api/site?name=shadow").get("knobs"), "sets": ["GATE_SHADOW=True"]})
    res["config"] = get("/api/config")
    res["paths"] = get("/api/paths")
    res["checks"] = get("/api/checks")
    try:
        res["launcher_temp"] = post("/api/launcher_temp", {"knobs": get("/api/site?name=sim").get("knobs"), "sets": []})
    except Exception as e:
        res["launcher_temp"] = repr(e)
    res["launcher"] = get("/api/launcher")
    fly = os.path.join(home, "fly.sh")
    res["fly_sh"] = open(fly).read() if os.path.exists(fly) else None
finally:
    p.terminate()
s = json.dumps(res, ensure_ascii=False, indent=1, sort_keys=True)
s = s.replace(home, "<HOME>").replace(os.path.abspath(root), "<ROOT>")
s = re.sub(r'"(t|t_gen|mtime|age|gen_age|time|ts|t_state|now)": [0-9.e+-]+', r'"\1": 0', s)
s = re.sub(r"\d{4}-\d\d-\d\d \d\d:\d\d:\d\d", "<DATE>", s)
s = re.sub(r"\b1\d{9}\b", "<EPOCH>", s)
open(out, "w").write(s)
print("wrote", out, len(s))
