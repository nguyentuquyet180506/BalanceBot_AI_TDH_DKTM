@echo off
title Nap firmware ESP CAM (stream UDP + web + UART TX21 / RX14)
cd /d "%~dp0firmware\cam_esp"
echo ========================================================
echo   Nap firmware ESP32-S3 ESP CAM (stream UDP + web + UART TX21 / RX14)
echo   Cam CHI mot ESP vao may, hoac chi ro cong: upload_cam.bat COMx
echo ========================================================
if "%~1"=="" (
    pio run -t upload
) else (
    pio run -t upload --upload-port %1
)
if %ERRORLEVEL% equ 0 (
    echo.
    echo [+] Nap ESP CAM (stream UDP + web + UART TX21 / RX14) thanh cong!
) else (
    echo.
    echo [!] Nap that bai! Kiem tra cap USB / cong COM, hoac giu nut BOOT khi cam cap.
)
pause
