@echo off
chcp 65001 >nul
title BalanceBot - Can gat UDP
cd /d "%~dp0pc"
echo Cua so can gat UDP (xe tai 192.168.1.200, hoac: control.bat 192.168.4.1)
if exist "%USERPROFILE%\AppData\Local\Python\bin\python.exe" (
    "%USERPROFILE%\AppData\Local\Python\bin\python.exe" run.py --control %*
) else (
    python run.py --control %*
)
