# start_webcontrol.ps1 - PowerShell 1-Click Launcher for Teensy 4.1 Control Studio
$ErrorActionPreference = "Continue"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -Path $ScriptDir

Write-Host "========================================================================" -ForegroundColor Cyan
Write-Host "🦾  TEENSY 4.1 CONTROL STUDIO" -ForegroundColor Cyan
Write-Host "========================================================================" -ForegroundColor Cyan
Write-Host "Directory: $ScriptDir`n" -ForegroundColor DarkGray

# Check for Python
$PythonCmd = Get-Command python -ErrorAction SilentlyContinue
if (-not $PythonCmd) {
    $PythonCmd = Get-Command python3 -ErrorAction SilentlyContinue
}
if (-not $PythonCmd) {
    $PythonCmd = Get-Command py -ErrorAction SilentlyContinue
}

if ($PythonCmd) {
    Write-Host "[OK] Python detected ($($PythonCmd.Source)). Launching..." -ForegroundColor Green
    & $PythonCmd.Source launch.py
    exit $LASTEXITCODE
}

# Check for Node.js
$NodeCmd = Get-Command node -ErrorAction SilentlyContinue
if ($NodeCmd) {
    Write-Host "[OK] Node.js detected. Launching static web server..." -ForegroundColor Green
    Start-Process "http://localhost:8000/"
    npx --yes serve -l 8000 .
    exit $LASTEXITCODE
}

Write-Host "========================================================================" -ForegroundColor Red
Write-Host "[ERROR] Neither Python 3 nor Node.js was found in PATH!" -ForegroundColor Red
Write-Host "========================================================================" -ForegroundColor Red
Write-Host ""
Write-Host "Control Studio cannot be opened directly via the file:// protocol." -ForegroundColor Yellow
Write-Host "The application requires a local HTTP web server because it relies on:" -ForegroundColor Yellow
Write-Host "  - JavaScript ES6 Modules (type=`"module`")" -ForegroundColor Gray
Write-Host "  - Browser ESM Import Maps (importmap)" -ForegroundColor Gray
Write-Host "  - Dynamic 3D STL Mesh and Asset Loading (fetch API)" -ForegroundColor Gray
Write-Host ""
Write-Host "Modern browsers block these features on file:// URLs due to CORS security policies." -ForegroundColor Yellow
Write-Host ""
Write-Host "Please install one of the following runtimes to run the local server:" -ForegroundColor White
Write-Host "  1. Python 3 (recommended): https://www.python.org/downloads/" -ForegroundColor Cyan
Write-Host "     * Ensure you check `"Add python.exe to PATH`" during installation." -ForegroundColor DarkGray
Write-Host "  2. Node.js (LTS):          https://nodejs.org/" -ForegroundColor Cyan
Write-Host ""
Write-Host "After installation, restart this script." -ForegroundColor White
Write-Host "========================================================================" -ForegroundColor Red
Write-Host ""
Read-Host "Press Enter to exit..."
exit 1
