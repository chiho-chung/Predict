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

g++ -std=c++17 -O2 -Wall -Wextra -Isrc ^
  tools\rate_check.cpp src\sim.cpp src\predictor.cpp src\csv_log.cpp ^
  -o build\rate_check.exe -static
if errorlevel 1 exit /b 1

build\rate_check.exe
endlocal
