@echo off
REM -- Camera to Robot launcher ---------------------------------------------
REM Double-click this file to start the program.

REM Use UTF-8 so Unicode log output does not crash the program.
set PYTHONUTF8=1
chcp 65001 >nul

REM Move to the folder this script lives in, so it works from anywhere.
cd /d "%~dp0"

echo Starting Camera to Robot...
echo (A browser tab will open at http://localhost:8080)
echo.

python main.py

REM Keep the window open after the program exits so any error stays visible.
echo.
echo Program stopped. Press any key to close this window.
pause >nul
