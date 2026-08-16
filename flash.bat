@echo off
rem Doppelklick-Starter fuer flash.ps1 – kompiliert und ueberträgt die Firmware.
rem Anderen Port: flash.bat COM5

setlocal
set PORT=%~1
if "%PORT%"=="" set PORT=COM3

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" -Port %PORT%

echo.
pause
