@echo off
rem start_gcs.bat —— 起笔记本地面检测（gcs_gate）。参数 1 = 模式(1=SIM 2=REAL)
cd /d "%~dp0..\_ref\s14" || (echo 找不到 _ref\s14 & exit /b 1)
set "PYTHONIOENCODING=gbk:replace"
if "%~1"=="" (set GMODE=1) else (set GMODE=%~1)
if not exist gcs_gate.py (echo 找不到 gcs_gate.py & exit /b 1)
"C:\miniforge3\envs\mcpv\python.exe" -u gcs_gate.py --drone 192.168.2.20 --mode %GMODE% --target reddot --headless --scale 1.0 --preview-width 1920 > "%TEMP%\gcs_gate.log" 2>&1