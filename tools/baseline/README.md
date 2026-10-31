# 等价验证基线 (PLAN_refactor_v2 §3)

    python3 tools/baseline/run_all.py            # 全部 (~70 s)
    python3 tools/baseline/run_all.py --quick    # 跳过 det_replay 和 sim (~10 s)

| 脚本 | 比什么 | 判定 |
|---|---|---|
| dryrun_dump.py | `sites/*.conf` + 几组命令行 → `<入口> --dry-run [--json]` | 逐字节 (路径换成 `<ROOT>`; 戳点字段/行两边都去掉) |
| plan_grid.py | `plan_dash` / `preflight_check` / `static_preview` 参数网格 | float repr 相同 (Plan 的戳点属性白名单) |
| det_replay.py | 合成双目序列 (`synth.py`) → `GateDetector.process` → `frame_to_msg`; 机上路径 (带 T_WC) | 逐字段 (`t_ms` 除外) |
| onboard_replay.py | 同一合成序列放大成 3840×1080 并排帧 + 位姿 → `vision_bridge.GateWorker.step` (机上检测模式) → `/gate/status` + 预览 | 逐字段 (`ms` / `det_ms` 除外) |
| fuse_replay.py | 同一批地面观测 + 位姿 → `vision_bridge.GateFuseWorker.on_obs` → `/gate/status` | 逐字段 (`ms` / `det_ms` 除外); 3 个变体 |
| judge_replay.py | `/gate/status` 序列 + 人工坏帧 → `GateInput.judge` → `gp.fuse_samples` | 样本 / 拒绝原因 / 粗估相同 |
| sim_check.py | `sim/run_sim.py` 10 个场景并行 | 结构精确 + 数值容差 (仿真按墙钟走) |

- golden 在 **存档版本** (`git` 第一个提交) 上用 `--write` 生成, 解释器 Python 3.13 / numpy 2.5 / OpenCV 5.0。
  换解释器会有 1 ulp 的浮点差 (3.8 上 judge 的 sig_hva 末位不同) —— 比较时用同一个解释器。
- 被测代码的位置只在 `_compat.py` 里写; 重构搬家时改它, 各脚本不动。
- 还没有实录: det_replay 只有合成序列。拿到 `gcs_gate --record` 的录像后加 `--rec` 输入。
- sim 失败时完整输出在 `tools/baseline/out/sim_<场景>.log`。
