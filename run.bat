@echo off
chcp 65001 >nul
title BalanceBot - camera UDP + AI + dieu khien
cd /d "%~dp0pc"
echo ========================================================
echo   ROBOT CAN BANG + AI BAM NGUOI (may tinh)
echo   Doi che do: go 1 / 2 / 3 / 4 roi Enter trong cua so nay
echo     1 tay ^| 2 cu chi ^| 3 bam nguoi ^| 4 xem cam
echo   Che do 2/3: tim chu bang webcam, bam Y de chuyen sang cam ESP
echo   Vi du: run.bat --mode 3     run.bat 192.168.4.1
echo ========================================================
if exist "%USERPROFILE%\AppData\Local\Python\bin\python.exe" (
    "%USERPROFILE%\AppData\Local\Python\bin\python.exe" run.py  %*
) else (
    python run.py  %*
)
if errorlevel 1 pause
