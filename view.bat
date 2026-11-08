@echo off
chcp 65001 >nul
title BalanceBot - Chi xem camera
cd /d "%~dp0pc"
if exist "%USERPROFILE%\AppData\Local\Python\bin\python.exe" (
    "%USERPROFILE%\AppData\Local\Python\bin\python.exe" run.py --view %*
) else (
    python run.py --view %*
)
pause
