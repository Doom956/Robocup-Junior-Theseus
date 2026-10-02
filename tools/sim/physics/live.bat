@echo off
rem Double-click to open the live view: several simulated robots solving random mazes at once.
rem Keep this window open while you use it. Close it (or press Ctrl+C) to stop the robots.
cd /d "%~dp0"
python run_physics.py --live %*
if errorlevel 1 pause
