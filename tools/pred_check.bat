@echo off
setlocal
cd /d "%~dp0.."

set "MSYS=C:\msys64\ucrt64\bin"
if not exist "%MSYS%\g++.exe" (
  echo g++ not found at %MSYS%\g++.exe
  exit /b 1
)
set "PATH=%MSYS%;%PATH%"

if not exist build mkdir build

g++ -std=c++17 -O2 -Wall -Wextra -Isrc -Iexport/bbox_ekf -Iexport/bbox_imm_ekf ^
  tools\pred_check.cpp src\sim\sim.cpp src\estimator\predictor.cpp src\io\csv_log.cpp ^
  export\bbox_ekf\bbox_ekf.cpp export\bbox_imm_ekf\bbox_imm_ekf.cpp ^
  -o build\pred_check.exe -static
if errorlevel 1 exit /b 1

build\pred_check.exe
endlocal
