@echo off
setlocal
cd /d "%~dp0"
echo Starting Motor Test Workbench. Keep this window open while using the site.
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start.ps1"
if errorlevel 1 (
    echo.
    echo The workbench could not start. Read the message above, then press any key to close.
    pause >nul
)
