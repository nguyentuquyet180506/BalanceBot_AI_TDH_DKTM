@echo off
title Serial Monitor - ESP CAM (cam vao cong TTL)
cd /d "%~dp0firmware\cam_esp"
if "%~1"=="" (pio device monitor -b 115200) else (pio device monitor -b 115200 -p %1)
pause
