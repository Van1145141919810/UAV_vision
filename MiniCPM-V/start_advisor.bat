@echo off
chcp 936 <nul >nul 2>&1
cd /d "%~dp0"
set "PYTHONIOENCODING=gbk:replace"
title VLM 方向顾问 - 只给建议，不控制飞机
set "PY=C:\miniforge3\envs\mcpv\python.exe"
if not exist "%PY%" set "PY=python"

echo ============================================================
echo   VLM 方向顾问（只画建议，不控制飞机）
echo ============================================================
echo.
set "PORT=8093"
set /p PORT=  端口（回车=8093；想占旧地址就输 8092）:
if "%PORT%"=="" set "PORT=8093"
echo.
echo   远距门限：距离大于等于多少米时，一律给"往哪飞"的方向建议？
echo     回车 = 不启用（按 CV 状态自动决定问什么）
echo     常用 = 15（推荐：15 m 以外都让它指方向）
set "FAR="
set /p FAR=  米数（回车=不启用）:
echo.
echo   网页「识别画面 [地址]」填：
echo       http://127.0.0.1:%PORT%/snapshot.jpg
echo   调试信息：  http://127.0.0.1:%PORT%/advice.json
echo.
echo   ★ 它已包含"选编号"，不要再开 start_vlm.bat，两个会抢显存。
echo   关掉这个窗口 = 顾问下线，飞行链路一点不受影响。
echo.
if "%FAR%"=="" (
  "%PY%" -u scripts\vlm_advisor.py --port %PORT% --mode auto
) else (
  "%PY%" -u scripts\vlm_advisor.py --port %PORT% --mode auto --far-m %FAR%
)
echo.
echo 顾问已退出（退出码 %ERRORLEVEL%）
pause