@echo off
setlocal
cd /d "%~dp0"

set "MSYS=C:\msys64\ucrt64\bin"
if not exist "%MSYS%\g++.exe" (
  echo g++ not found at %MSYS%\g++.exe
  exit /b 1
)
set "PATH=%MSYS%;%PATH%"

if not exist build mkdir build

echo Compiling drone_chase_sim...
g++ -std=c++17 -O2 -Wall -Wextra -Isrc ^
  src\main.cpp src\sim.cpp src\render.cpp src\predictor.cpp src\plot.cpp src\csv_log.cpp src\bench.cpp ^
  -o build\drone_chase_sim.exe ^
  -static -static-libgcc -static-libstdc++ ^
  -lgdi32 -luser32
if errorlevel 1 (
  echo Build failed.
  exit /b 1
)

echo OK: build\drone_chase_sim.exe
echo Manual: build\drone_chase_sim.exe
echo Auto:   build\drone_chase_sim.exe auto
endlocal
