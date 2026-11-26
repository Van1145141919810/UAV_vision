@echo off
chcp 936 >nul
cd /d "%~dp0"
title 破窗前置 - 红点跟踪

set "PY=C:\miniforge3\envs\mcpv\python.exe"
if not exist "%PY%" set "PY=python"
set PYTHONPATH=%CD%\src;%CD%\scripts

echo ============================================================
echo   破窗前置：红点跟踪 -^> 接近 -^> 交接
echo ============================================================
echo   连续跟踪版本（不是点击式 demo）
echo.
echo   1 = 测视频流（先跑这个：看分辨率/帧率，定红圆尺寸）
echo   2 = dryrun（只算指令，不发给飞机）  ^<-^- 你现在要的
echo   3 = 真飞（螺旋桨已装、场地清空）
echo.
set /p MODE=输入 1 / 2 / 3 后回车(直接回车=1):
if "%MODE%"=="" set MODE=1

if "%MODE%"=="1" goto probe
if "%MODE%"=="2" goto dry
if "%MODE%"=="3" goto live
echo 输入无效
pause
exit /b 1

:probe
echo.
echo ---------- 测视频流 ----------
"%PY%" scripts\probe_stream.py
echo.
echo 把这里的"分辨率"和"交接点 d_px"告诉我，我据此确认红圆尺寸。
pause
exit /b 0

:dry
echo.
echo ---------- dryrun：只算指令，不发给飞机 ----------
echo   操作：单击选候选 / 回车确认（无候选时=朝该点步进5m）/
echo         +/- 调步长 / B 连续盲进 / G 扫视 / WASD RF QE 接管 /
echo         T 恢复 / C 释放目标重选 / 空格急停 / X 退出 / L L 降落
echo.
"%PY%" scripts\breach_run.py --dry
echo.
pause
exit /b 0

:live
echo.
echo ============================================================
echo   真飞模式
echo ============================================================
echo   确认：螺旋桨已装 / 场地清空 / 电池 ^>15V / 你能随时接管
echo.
"%PY%" scripts\breach_run.py --live
echo.
pause
exit /b 0
