@echo off
title Serial Monitor - ESP CAN BANG
cd /d "%~dp0firmware\balance_esp"
if "%~1"=="" (pio device monitor -b 115200) else (pio device monitor -b 115200 -p %1)
pause
