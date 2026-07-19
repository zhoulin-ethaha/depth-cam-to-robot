@echo off
REM -- Dual-camera stitching prototype launcher ------------------------------
REM Merges the feeds of TWO RealSense D435i cameras into one heightmap.
REM CONTAINED prototype: close the main app first (one process per RealSense).
REM With fewer than two cameras it runs on a synthetic scene.

set PYTHONUTF8=1
chcp 65001 >nul
cd /d "%~dp0"

echo Starting Dual-Camera Stitch prototype...
echo (A browser tab will open at http://localhost:5006)
echo.

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" stitch_main.py
) else (
    echo WARNING: .venv not found - using PATH python. Create it with:
    echo     python -m venv .venv ^&^& .venv\Scripts\python.exe -m pip install -r requirements.txt
    echo.
    python stitch_main.py
)

echo.
echo Program stopped. Press any key to close this window.
pause >nul
