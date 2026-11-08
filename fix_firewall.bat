@echo off
title Open UDP Port 4210 for BalanceBot
echo ========================================================
echo   Mo cong UDP 4210 tren Windows Firewall cho BalanceBot
echo ========================================================
echo.
netsh advfirewall firewall add rule name="BalanceBot UDP 4210 In" dir=in action=allow protocol=UDP localport=4210
netsh advfirewall firewall add rule name="BalanceBot UDP 4210 Out" dir=out action=allow protocol=UDP localport=4210
echo.
echo [+] Da them Rule cho phep UDP 4210 vao Windows Firewall thanh cong!
echo     Neu van khong nhan duoc hinh / phan hoi: cho phep python.exe trong Windows Firewall (mang Private).
pause
