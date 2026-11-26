@echo off
rem start_vlm.bat —— VLM 第二意见（建议画到 8092 的图上给网页看）
cd /d "%~dp0"
set "PYTHONIOENCODING=gbk:replace"
"C:\miniforge3\envs\mcpv\python.exe" -u scripts\vlm_bridge.py --serve-port 8092
