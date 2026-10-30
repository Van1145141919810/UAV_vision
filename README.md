# VB09 穿环打击 · 代码存档 2026-10-30

文档：docs/VB09_gate_system_guide.pdf（在线版 https://claude.ai/artifact/TCw68ZtrMRDnAPFB3ethyi）

| 文件 | 部署位置 | 作用 |
| --- | --- | --- |
| stage12_gate.py | 飞机 | 飞行脚本：预检、状态机、轨迹下发、安全监控、日志 |
| gate_plan.py | 飞机 | 打击规划（纯函数，dry-run / 网页预览共用） |
| flight_profile.py | 飞机 | 现场档案解析与校验 |
| flight_monitor.py + monitor.html | 飞机 | 监控网页、机况检查、生成 fly.sh、回放 |
| run_vision.py + vision_bridge.py | 飞机 | 图传桥：采图、位姿插值、世界系融合、/gate/status |
| gcs_gate.py | 地面站 | 地面检测 |
| gate_detector.py + ring_detector.py | 飞机 + 地面站 | 检测库 |

未包含（不在本次上传中）：stereo_calib.yaml、gate_params.json、preview_server.py、~/flight_sites/*.conf。

注：stage12_gate.py 文件头仍写 stage13，是同一份脚本改名（飞机启动脚本按文件名查找）。

## SHA-256
    b37bbe75c550a92af89f68eff7647aa5c923b73acb406258b72613a3488568c9  flight_monitor.py
    8bb30d3e20cd77119fa8b08075cd72518b98bdc618105e8b35c13947db6f29ab  flight_profile.py
    e0a0b263103a999b0d5e6fc7ab2e19213e6e0f62acf7e11caf67dd28aeb4315a  gate_detector.py
    dc77813e0226b4150a57a625e5e1716b322eb9b2eb4e627a83aaa21a6b1d6ac7  gate_plan.py
    b18621229f8e5c453b3d8e1377a413136f4f9b94193db21198d50beb0303da26  gcs_gate.py
    a90e9afa0e0e9cac74f051e2abbfaa401c7b51fb2f04b527dc752213bc0e009b  monitor.html
    3231a31465b050182029fed7edad97382cf02ae2859e72707fdae33e2f7229a6  ring_detector.py
    da50135317ff2b37f5bf427c3a2772cd296b77e4b602e0732dd6f392076cfb73  run_vision.py
    6f5d88a55fc253f89835f31ca06935f9ec80d43f249ac382a5cd21ff86ef9163  stage12_gate.py
    a848a3b04c5e0b9244dde75ba72bd2ae6fb4caa4885a611bd72866915e4ece6e  vision_bridge.py
