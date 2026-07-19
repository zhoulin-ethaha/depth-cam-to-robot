@echo off
REM -- Saved-toolpath replay launcher ----------------------------------------
REM Connect the robot, pick a saved bundle under paths\ and re-run it.
REM CONTAINED tool: close the main app first if it is connected to the robot
REM (one RTDE controller per robot). No camera needed.

set PYTHONUTF8=1
chcp 65001 >nul
cd /d "%~dp0"

echo Starting Toolpath Replay...
echo (A browser tab will open at http://localhost:5007)
echo.

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" replay_main.py
) else (
    echo WARNING: .venv not found - using PATH python. Create it with:
    echo     python -m venv .venv ^&^& .venv\Scripts\python.exe -m pip install -r requirements.txt
    echo.
    python replay_main.py
)

echo.
echo Program stopped. Press any key to close this window.
pause >nul
