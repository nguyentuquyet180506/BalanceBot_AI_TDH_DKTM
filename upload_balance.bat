@echo off
title Nap firmware ESP CAN BANG (khong cam, UART TX21 / RX42)
cd /d "%~dp0firmware\balance_esp"
echo ========================================================
echo   Nap firmware ESP32-S3 ESP CAN BANG (khong cam, UART TX21 / RX42)
echo   Cam CHI mot ESP vao may, hoac chi ro cong: upload_balance.bat COMx
echo ========================================================
if "%~1"=="" (
    pio run -t upload
) else (
    pio run -t upload --upload-port %1
)
if %ERRORLEVEL% equ 0 (
    echo.
    echo [+] Nap ESP CAN BANG (khong cam, UART TX21 / RX42) thanh cong!
) else (
    echo.
    echo [!] Nap that bai! Kiem tra cap USB / cong COM, hoac giu nut BOOT khi cam cap.
)
pause
