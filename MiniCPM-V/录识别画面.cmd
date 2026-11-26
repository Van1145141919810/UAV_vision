@echo off
chcp 936 <nul >nul 2>&1
cd /d "%~dp0"
title 录识别画面 - 交差/分析用
set "PY=C:\miniforge3\envs\mcpv\python.exe"
if not exist "%PY%" set "PY=python"

echo ============================================================
echo   录「识别画面」（在笔记本上录，不占飞机 CPU）
echo ============================================================
echo   1  带建议画面  :8093  ← 开了方向顾问时用（画面里有编号/建议）
echo   2  原样画面    :8091  ← 没开顾问时用
echo.
set "S=."
set /p S=  录哪个？(1/2，回车=1):
if "%S%"=="2" (set "SRC=8091") else (set "SRC=8093")
echo.
echo   录多久？输秒数；直接回车 = 一直录，按 Ctrl+C 停
set "SEC=0"
set /p SEC=  秒数(回车=一直录):
if "%SEC%"=="" set "SEC=0"
echo.
echo   存到 D:\Desktop\无人系统\视频\
echo   录完会自己把文件重新打开读一遍，打「自检 OK」才算成。
echo.
"%PY%" -u scripts\record_view.py --src %SRC% --seconds %SEC%
echo.
pause