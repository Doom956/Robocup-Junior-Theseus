@echo off
rem Double-click to open the batch dashboard: run hundreds of fields and compare the results.
rem Keep this window open while you use it. Close it (or press Ctrl+C) to stop.
cd /d "%~dp0"
python run_physics.py --serve %*
if errorlevel 1 pause
