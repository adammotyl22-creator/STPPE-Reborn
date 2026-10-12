@echo off
setlocal enabledelayedexpansion

title Teensy 4.1 Robot Arm Automation Studio
cd /d "%~dp0"

echo ========================================================================
echo 🦾  TEENSY 4.1 ROBOT ARM AUTOMATION STUDIO (WEBCONTROL)
echo ========================================================================
echo Initializing standalone web server...
echo.

:: Check for Python 3
where python >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [OK] Python detected. Starting high-performance launcher...
    python launch.py
    goto end
)

where python3 >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [OK] Python3 detected. Starting high-performance launcher...
    python3 launch.py
    goto end
)

:: Check for Node.js
where node >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [OK] Node.js detected. Launching static web server on port 8000...
    start "" http://localhost:8000/
    npx --yes serve -l 8000 .
    goto end
)

echo [WARNING] Neither Python nor Node.js was found in PATH.
echo Attempting to open index.html directly in default browser...
start "" index.html

:end
pause
