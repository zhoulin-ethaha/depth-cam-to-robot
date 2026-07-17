@echo off
REM -- Depth Camera to Robot launcher ---------------------------------------
REM Double-click this file to start the program.

REM Use UTF-8 so Unicode log output does not crash the program.
set PYTHONUTF8=1
chcp 65001 >nul

REM Move to the folder this script lives in, so it works from anywhere.
cd /d "%~dp0"

echo Starting Depth Camera to Robot (Developer Mode)...
echo (A browser tab will open at http://localhost:5005)
echo.

REM Use the project's virtual environment so pyrealsense2 and the other pinned
REM dependencies are found. Falls back to the PATH python only if no venv exists.
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" main.py
) else (
    echo WARNING: .venv not found - using PATH python. Create it with:
    echo     python -m venv .venv ^&^& .venv\Scripts\python.exe -m pip install -r requirements.txt
    echo.
    python main.py
)

REM Keep the window open after the program exits so any error stays visible.
echo.
echo Program stopped. Press any key to close this window.
pause >nul
