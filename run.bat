@echo off
chcp 65001 >nul
title BalanceBot - camera UDP + AI + dieu khien
cd /d "%~dp0pc"
echo ========================================================
echo   ROBOT CAN BANG + AI BAM NGUOI (may tinh)
echo   Doi che do: go 1 / 2 / 3 / 4 / 5 roi Enter trong cua so nay
echo     1 tay ^| 2 cu chi ^| 3 xem cam ^| 4 bam nguoi ao+quan ^| 5 bam nguoi ao+quan+tay ao khac mau
echo   Che do 2/4/5: tim chu bang webcam, bam Y de chuyen sang cam ESP
echo   Vi du: run.bat --mode 5     run.bat 192.168.4.1
echo ========================================================
if exist "%USERPROFILE%\AppData\Local\Python\bin\python.exe" (
    "%USERPROFILE%\AppData\Local\Python\bin\python.exe" run.py  %*
) else (
    python run.py  %*
)
if errorlevel 1 pause
