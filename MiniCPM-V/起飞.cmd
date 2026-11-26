@echo off
chcp 936 <nul >nul 2>&1
cd /d "%~dp0"
title 起飞助手 - 地面检测模式
setlocal EnableExtensions

set "KEY=%USERPROFILE%\.ssh\dsh_drone"
set "DRONE=visbot@192.168.2.20"
set "PY=C:\miniforge3\envs\mcpv\python.exe"
set "HF_HOME=%~dp0.hf"
set "HF_ENDPOINT=https://hf-mirror.com"
set "S14L=%~dp0..\_ref\s14"
set "SSH=ssh -i "%KEY%" -o StrictHostKeyChecking=no -o ConnectTimeout=8 %DRONE%"
set "ADV_PORT=8093"

rem 笔记本 IP：默认 192.168.2.179，可用同目录 gcs_ip.txt 覆盖
set "GCSIP=192.168.2.179"
if exist "%~dp0gcs_ip.txt" for /f "usebackq delims=" %%i in ("%~dp0gcs_ip.txt") do set "GCSIP=%%i"
if "%GCSIP%"=="" set "GCSIP=192.168.2.179"

rem ============================================================
rem  菜单
rem ============================================================
:menu
cls
call :probe
echo.
echo  ============================================================
echo    起飞助手   地面检测模式（笔记本跑检测 + 大模型给建议）
echo  ============================================================
echo     笔记本 %GCSIP%        飞机 192.168.2.20
echo     现在在跑： 检测 %P8091%   隧道 %P8080%   编号建议 %P8092%   方向顾问 %P8093%
echo  ============================================================
echo   【一、准备】  （每次开机先做这两步）
echo     1  环境检查      ping / SSH / 谁在发设定点
echo     2  准备飞行      一键起好：关抢占 + 飞机桥 + 检测 + 监控 + 网页
echo.
echo   【二、飞之前】  （可选，但建议至少跑一次 3）
echo     3  参数预览      dry-run：只打印生效参数，不飞
echo     6  流程演练      sim：不接飞机，练手（约 45 秒）
echo.
echo   【三、飞】
echo     4  起飞          飞行控制台（I 看 / U 走 / Esc 退出；弹新窗口，输入 yes 确认）
echo.
echo   【四、看】
echo     8  看状态        卡住时先看这个：回包 Hz / 检测日志 / tmux
echo     7  看监控网页    重开隧道和浏览器
echo     9  编号建议      旧 VLM：在候选里挑一个（可选）
echo     A  方向顾问      新 VLM：给"往哪边飞"的方向（可选，端口 %ADV_PORT%）
echo.
echo   【五、收尾】
echo     5  收尾          飞完/收工：停飞机 + 检测 + 隧道 + 所有 VLM
echo  ============================================================
echo     0  退出
echo.
set "M="
set /p M=  请输入编号后回车（默认 1）:
if not defined M set "M=1"
if "%M%"=="0" goto bye
if "%M%"=="1" goto check
if "%M%"=="2" goto prep
if "%M%"=="3" goto dryrun
if "%M%"=="4" goto fly
if "%M%"=="5" goto stop
if "%M%"=="6" goto rehearse
if "%M%"=="7" goto web
if "%M%"=="8" goto status
if "%M%"=="9" goto vlm
if /i "%M%"=="A" goto adv
echo.
echo   输入「%M%」不认识。请按上面列出的编号/字母。
ping -n 3 127.0.0.1 >nul
goto menu

rem ============================================================
rem  1. 环境检查
rem ============================================================
:check
echo.
echo  ========== 1. 连通性 ==========
ping -n 1 -w 1500 192.168.2.20 | findstr /C:"TTL" >nul
if errorlevel 1 goto noping
echo    OK  ping 通（本机 %GCSIP% --^> 飞机 192.168.2.20）
%SSH% "echo    OK SSH 通"
if errorlevel 1 goto nossh
echo.
echo  ========== 2. 谁在发设定点 ==========
%SSH% "bash ~/cpmvbyhuang/free_setpoints.sh --status"
echo.
echo    期望看到  Publishers: None
echo    如果有人在发（Publishers 后面不是 None），回到菜单按 2 准备飞行会自动关掉。
echo.
pause & goto menu

:noping
echo.
echo    x  ping 不通 192.168.2.20
echo.
echo    按顺序检查：
echo      1) 笔记本 WiFi 是否连上同一台飞机热点（不是校园网 / 不是手机热点）
echo      2) 飞机的电源和网卡是否已上电
echo      3) 本机 IP 是不是 %GCSIP%（不是就用记事本改 gcs_ip.txt）
echo.
pause & goto menu

:nossh
echo.
echo    x  SSH 连不上（ping 通但 SSH 不通）
echo.
echo      1) 先按 5 收尾，再重试
echo      2) 还不行就 ssh -i "%KEY%" %DRONE% 手动试一次，看报什么
echo.
pause & goto menu

rem ============================================================
rem  2. 准备飞行
rem ============================================================
:prep
echo.
echo  ============================================================
echo   准备飞行：一次把 5 件事做完（约 20 秒）
echo  ============================================================
echo.
echo  [1/5] 关闭抢占设定点的栈（保留 VINS）
%SSH% "bash ~/cpmvbyhuang/free_setpoints.sh"
echo.
echo  [2/5] 飞机桥切到 gate_gcs，回包发回笔记本 %GCSIP%
%SSH% "bash ~/cpmvbyhuang/start_vision_gcs.sh %GCSIP%"
echo.
:picksim
echo  ------------------------------------------------------------
echo    这一步选"飞机要不要跟我动"：
echo      1 = 只检测不驱动（SIM，练手/只看画面，飞机不会动）
echo      2 = 驱动飞机（REAL，真飞用这个）
echo  ------------------------------------------------------------
set "MODE="
set "GMODE="
set /p MODE=  选 1 或 2（直接回车 = 1）:
if not defined MODE set "MODE=1"
if "%MODE%"=="1" set "GMODE=1"
if "%MODE%"=="2" set "GMODE=2"
if not defined GMODE goto picksim_setdefault
goto picksim_done
:picksim_setdefault
set "GMODE=1"
:picksim_done
echo    地面模式 = --mode %GMODE%
echo.
echo  [3/5] 笔记本起地面检测（每目 1920x1080，--scale 1.0）
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -like '*gcs_gate*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" 2>nul
if not exist "%S14L%\gcs_gate.py" (echo    x 找不到 %S14L%\gcs_gate.py & pause & goto menu)
start "笔记本检测 - 别关" /min "%~dp0start_gcs.bat" %GMODE%
echo    已启动（最小化窗口，别关它）
timeout /t 8 /nobreak >nul
echo.
echo  [4/5] 飞机监控网页
%SSH% "bash ~/cpmvbyhuang/start_monitor.sh"
echo.
echo  [5/5] 打开网页
call :openweb
echo.
echo  ============================================================
echo   起好了。下面 4 条按顺序对一遍，全对再去按 4 起飞：
echo     1. 网页「机况检查」全绿（geometric_controller 未运行）
echo     2. 「识别画面」有图（来自笔记本 8091）
echo     3. 「起飞配置」选档案：演示优化
echo     4. 回到本菜单按 8，看回包 Hz 应为 10~15
echo  ============================================================
echo.
pause & goto menu

rem ============================================================
rem  3. 参数预览
rem ============================================================
:dryrun
echo.
echo  参数预览（dry-run）：只打印生效参数，不会让飞机动。
echo.
set "SITE="
set /p SITE=  档案名（直接回车 = 演示优化）:
if not defined SITE set "SITE=演示优化"
echo.
%SSH% "cd ~/program/UAV/src/uav_test/scripts/detect_track/s14 && python3 stage14_mission.py --site %SITE% --dry-run 2>&1 | tail -26"
echo.
echo  重点看三行：xy_radius / z_high / 阶段包络速度。数字不对就别飞。
echo.
pause & goto menu

rem ============================================================
rem  4. 起飞
rem ============================================================
:fly
echo.
echo  ============================================================
echo   起飞
echo  ============================================================
echo    前提（缺一步都飞不起来）：
echo      1) 网页里点过「生成 fly.sh」，而且没超过 60 秒
echo      2) 下面弹出的新窗口里，输入 yes 再回车
echo      3) 你手上有遥控器，随时能接管
echo.
echo    飞完记得回本菜单按 5 收尾，否则飞机上的进程一直挂着。
echo  ============================================================
echo    注入模式：
echo      1 = dry-run：U 只打印，绝不注入          ^<-- 先试这个
echo      2 = 真动：U 会真的注入按键（要再输 MOVE 确认）
echo  ============================================================
echo.
set "DM="
set /p DM=  选 1 / 2（直接回车 = 1）: 
if not defined DM set "DM=1"
if "%DM%"=="2" goto fly_move
start "飞行 - I 看 / U 走 / T 交接 / G 冲刺" cmd /k %PY% scripts\fly_console.py --live
echo.
echo    新窗口已弹出。这个菜单窗口留着别关，方便飞完收尾。
echo.
pause & goto menu

:fly_move
echo.
echo  *** 真动模式：U 会真的注入按键 ***
echo  确认：螺旋桨状态 / 场地清空 / 随时能按 L 降落、X 停桨
set "OK="
set /p OK=  输入 MOVE 回车确认（其它 = 取消）: 
if /i not "%OK%"=="MOVE" (echo    已取消 & ping -n 3 127.0.0.1 ^>nul & goto menu)
start "飞行 - I 看 / U 走（真动）/ T 交接 / G 冲刺" cmd /k %PY% scripts\fly_console.py --live --allow-move
echo.
echo    新窗口已弹出（真动模式）。这个菜单窗口留着别关。
echo.
pause & goto menu

rem ============================================================
rem  5. 收尾
rem ============================================================
:stop
echo.
echo  ============================================================
echo   收尾：把所有东西停掉
echo  ============================================================
echo.
echo  [1/4] 停笔记本上的 飞行控制台 + VLM（编号建议 / 方向顾问）
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -like '*fly_console*' -or $_.CommandLine -like '*winseg_worker*' -or $_.CommandLine -like '*vlm_bridge*' -or $_.CommandLine -like '*vlm_advisor*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" 2>nul
echo    已停（本来没在跑就忽略）
echo.
echo  [2/4] 停飞机上的三个进程
%SSH% "pkill -f vision_bridge.py 2>/dev/null; pkill -f run_vision.py 2>/dev/null; pkill -f flight_monitor.py 2>/dev/null; tmux kill-session -t vision 2>/dev/null; tmux kill-session -t monitor 2>/dev/null; echo    飞机端已停"
echo.
echo  [3/4] 停笔记本检测
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -like '*gcs_gate*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force; '    已停 PID ' + $_.ProcessId }" 2>nul
echo.
echo  [4/4] 关监控隧道
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='ssh.exe'\" | Where-Object { $_.CommandLine -like '*-L 8080*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force; '    已关 PID ' + $_.ProcessId }" 2>nul
echo.
call :probe
echo  ============================================================
echo   现在还在跑： 检测 %P8091%   隧道 %P8080%   编号建议 %P8092%   方向顾问 %P8093%
echo   （全是「·」就干净了）
echo  ============================================================
echo.
pause & goto menu

rem ============================================================
rem  6. 流程演练
rem ============================================================
:rehearse
echo.
echo  流程演练：在飞机上用仿真跑一遍（不接真实飞机，约 45 秒）
echo.
echo    1 = dock_reddot（停靠：识别 → 靠近 → 停住）
echo    2 = dash_reddot（冲刺：识别 → 摆位 → 冲过去）
echo    3 = 桌面演练（不连飞机，拿录像帧试 I / U，Esc 退出）
echo.
set "S="
set /p S=  选 1 / 2 / 3（直接回车 = 1）: 
if not defined S set "S=1"
if "%S%"=="3" goto rehearse_off
if "%S%"=="2" (set SC=dash_reddot) else (set SC=dock_reddot)
echo.
echo  场景 %SC%：下面会一直滚日志，跑完自己停，别按 Ctrl+C。
echo.
%SSH% "cd ~/program/UAV/src/uav_test/scripts/detect_track/s14 && timeout 200 python3 -u sim/run_sim.py %SC% --set GATE_DOCK_HOLD_S=3.0 --set GATE_REFINE_WAIT_S=0.5 2>&1"
echo.
pause & goto menu

:rehearse_off
echo.
echo  桌面演练：按 I 出建议、按 U 只打印、N 换帧、Esc 退出
echo.
%PY% scripts\fly_console.py --offline
echo.
pause & goto menu

rem ============================================================
rem  7. 看监控网页
rem ============================================================
:web
echo.
echo  重开监控网页（会先关掉旧隧道，再开新的）
echo.
call :openweb
echo.
pause & goto menu

rem ============================================================
rem  8. 看状态
rem ============================================================
:status
echo.
echo  ========== 飞机桥日志（看 上行 / 回包 Hz）==========
%SSH% "tail -4 /tmp/vision_gcs.log 2>/dev/null || tail -4 /tmp/vision.log 2>/dev/null"
echo.
echo  ========== 笔记本检测（最近 3 条统计）==========
if exist "%TEMP%\gcs_gate.log" (
  powershell -NoProfile -Command "Get-Content -Path '%TEMP%\gcs_gate.log' -Encoding Default -Tail 12 | Where-Object { $_ -match 'fps=|found=' } | Select-Object -Last 3"
) else (
  echo    没有日志（笔记本检测没起？回到菜单按 2）
)
echo.
echo  ========== 飞机上的 tmux 会话 ==========
%SSH% "tmux ls 2>/dev/null || echo    没有"
echo.
echo  怎么看：
echo    回包 Hz 是 0.0      -^> 飞机没收到回包，查 gcs_ip.txt 和笔记本 IP
echo    fps 低于 5          -^> 笔记本卡了，关掉浏览器别的标签页试试
echo    没有 found= 这行     -^> 检测还没跑起来，重按 2
echo.
pause & goto menu

rem ============================================================
rem  9. 编号建议（旧版 VLM）
rem ============================================================
:vlm
echo.
echo  ============================================================
echo   编号建议（旧版 VLM：在候选里挑一个，只打印不发指令）
echo  ============================================================
echo    一次性设置：网页「识别画面」右边的 [地址] 填
echo        http://127.0.0.1:8092/snapshot.jpg
echo    之后画面底部会出现绿字 VLM SUGGEST: #N
echo    你看着按数字 + T 即可；它说错了不采纳就行。
echo.
if not exist "%~dp0scripts\vlm_bridge.py" (echo    x 找不到 scripts\vlm_bridge.py & pause & goto menu)
start "VLM 编号建议" /min "%~dp0start_vlm.bat"
echo    已在后台启动（最小化窗口，可关）。
echo.
pause & goto menu

rem ============================================================
rem  A. 方向顾问（新版 VLM）
rem ============================================================
:adv
echo.
echo  ============================================================
echo   方向顾问（新版 VLM：远距给"往哪边飞"的方向）
echo  ============================================================
echo    它在 CV 一个候选都看不见时，告诉你目标大概在
echo      上/中/下 + 左/中/右，以及建议先前进几米。
echo    CV 一旦自己看见了，它会自动闭嘴。
echo.
echo    一次性设置：网页「识别画面」右边的 [地址] 填
echo        http://127.0.0.1:%ADV_PORT%/snapshot.jpg
echo    想看调试信息：浏览器打开 http://127.0.0.1:%ADV_PORT%/advice.json
echo.
echo    ★ 它已经包含"挑候选"，所以不要再同时按 9，两个会抢显存。
echo.
if not exist "%~dp0scripts\vlm_advisor.py" (echo    x 找不到 scripts\vlm_advisor.py & pause & goto menu)
start "VLM 方向顾问" /min "%~dp0start_advisor.bat" %ADV_PORT%
echo    已在后台启动（最小化窗口，可关）。
echo.
pause & goto menu

rem ============================================================
rem  子过程
rem ============================================================
:openweb
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='ssh.exe'\" | Where-Object { $_.CommandLine -like '*-L 8080*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" 2>nul
start "" /min ssh -i "%KEY%" -o StrictHostKeyChecking=no -o ExitOnForwardFailure=yes -N -L 8080:127.0.0.1:8080 %DRONE%
timeout /t 3 /nobreak >nul
start "" http://127.0.0.1:8080
echo    隧道已开（那个最小化 ssh 窗口别关），浏览器已打开 127.0.0.1:8080
exit /b 0

:probe
set "P8091=·"
set "P8080=·"
set "P8092=·"
set "P8093=·"
netstat -an | findstr /C:"LISTENING" | findstr /C:":8091 " >nul 2>&1 && set "P8091=在"
netstat -an | findstr /C:"LISTENING" | findstr /C:":8080 " >nul 2>&1 && set "P8080=在"
netstat -an | findstr /C:"LISTENING" | findstr /C:":8092 " >nul 2>&1 && set "P8092=在"
netstat -an | findstr /C:"LISTENING" | findstr /C:":8093 " >nul 2>&1 && set "P8093=在"
exit /b 0

:bye
call :probe
echo.
if "%P8091%%P8080%%P8092%%P8093%"=="········" (
  echo   已经全部停了，可以关窗口。
) else (
  echo   注意：还有东西在跑（上面标「在」的项），建议先按 5 收尾再关。
)
echo.
exit /b 0