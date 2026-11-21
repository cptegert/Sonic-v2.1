@echo off
rem Live log of a robot on Windows.
rem 1. Join the robot Wi-Fi first (Windows Wi-Fi menu): Shadow-log / Sonic-X-log / Knuckles-log, password sonicx123
rem 2. Run this file (double-click, or in a terminal: tools\log.bat)
rem Type a command (status, sensors, log) or a mark (!3 slower, !4 stuck, !5 stopped, !text) + Enter. q = quit.
rem Everything is saved to crashlogs\
cd /d "%~dp0.."
set PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe
if not exist "%PY%" set PY=python
echo Make sure this PC is connected to the robot Wi-Fi (e.g. Shadow-log, password sonicx123).
"%PY%" tools\wifi_log.py
pause
