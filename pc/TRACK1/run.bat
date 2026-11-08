@echo off
chcp 65001 >nul
title Robot AI Tracker - ESP32 UDP
echo ========================================================
echo   KHOI DONG HE THONG ROBOT AI BAM NGUOI + ESP32 CAM UDP
echo ========================================================
python ..\run.py %*
if errorlevel 1 (
    echo.
    echo [!] Da co loi xay ra hoac chuong trinh bi dung.
    pause
)
