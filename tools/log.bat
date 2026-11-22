@echo off
rem Robot dashboard on Windows (live log, marks, laps, track, speed calibration).
rem 1. Join the robot Wi-Fi first (Windows Wi-Fi menu): Shadow-log / Sonic-X-log / Knuckles-log, password sonicx123
rem 2. Run this file (double-click, or in a terminal: tools\log.bat shadow)
rem    The dashboard opens in the browser: http://localhost:8770
rem    Old text viewer instead:  tools\log.bat shadow term
rem Everything is saved to crashlogs\
cd /d "%~dp0.."
set PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe
if not exist "%PY%" set PY=python
set ROBOT=%1
if "%ROBOT%"=="" set ROBOT=sonic
echo Make sure this PC is connected to the robot Wi-Fi (e.g. Sonic-X-log, password sonicx123).
if "%2"=="term" (
  "%PY%" tools\wifi_log.py
) else (
  "%PY%" tools\dashboard.py --robot %ROBOT%
)
pause
