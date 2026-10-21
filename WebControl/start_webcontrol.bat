@echo off
setlocal enabledelayedexpansion

title Teensy 4.1 Control Studio
cd /d "%~dp0"

echo ========================================================================
echo 🦾  TEENSY 4.1 CONTROL STUDIO
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

where py >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [OK] Python (py launcher) detected. Starting high-performance launcher...
    py launch.py
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

echo ========================================================================
echo [ERROR] Neither Python 3 nor Node.js was found in PATH!
echo ========================================================================
echo.
echo Control Studio cannot be opened directly via the file:// protocol.
echo The application requires a local HTTP web server because it relies on:
echo   - JavaScript ES6 Modules (type="module")
echo   - Browser ESM Import Maps (importmap)
echo   - Dynamic 3D STL Mesh and Asset Loading (fetch API)
echo.
echo Modern browsers block these features on file:// URLs due to CORS security policies.
echo.
echo Please install one of the following runtimes to run the local server:
echo   1. Python 3 (recommended): https://www.python.org/downloads/
echo      * Ensure you check "Add python.exe to PATH" during installation.
echo   2. Node.js (LTS):          https://nodejs.org/
echo.
echo After installation, restart this script.
echo ========================================================================
echo.

:end
pause
