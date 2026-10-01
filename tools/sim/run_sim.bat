@echo off
rem Builds the maze simulator against the real Main\ navigation code, then runs it.
rem Arguments go to the simulator, e.g.:
rem   tools\sim\run_sim.bat
rem   tools\sim\run_sim.bat --runs 2000
rem   tools\sim\run_sim.bat --scenario ramp --seed 7 --show
rem Needs the PC compiler once:  pio pkg install -g --tool platformio/toolchain-gccmingw32
setlocal
set "HERE=%~dp0"
set "MAIN=%HERE%..\..\Main"
set "TOOLBIN=%USERPROFILE%\.platformio\packages\toolchain-gccmingw32\bin"
if exist "%TOOLBIN%\g++.exe" set "PATH=%TOOLBIN%;%PATH%"
where g++ >nul 2>nul || (echo g++ not found. Run: pio pkg install -g --tool platformio/toolchain-gccmingw32 & exit /b 1)
g++ -std=gnu++14 -O2 -Wall -static -I "%HERE%stubs" -I "%MAIN%" "%HERE%sim.cpp" "%MAIN%\navigation.cpp" "%MAIN%\MazeTile.cpp" -o "%HERE%sim.exe" || exit /b 1
"%HERE%sim.exe" %*
