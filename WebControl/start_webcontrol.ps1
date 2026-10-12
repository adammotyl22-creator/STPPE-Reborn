# start_webcontrol.ps1 - PowerShell 1-Click Launcher for Teensy 4.1 Robot Arm Automation Studio
$ErrorActionPreference = "Continue"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -Path $ScriptDir

Write-Host "========================================================================" -ForegroundColor Cyan
Write-Host "🦾  TEENSY 4.1 ROBOT ARM AUTOMATION STUDIO (WEBCONTROL)" -ForegroundColor Cyan
Write-Host "========================================================================" -ForegroundColor Cyan
Write-Host "Directory: $ScriptDir`n" -ForegroundColor DarkGray

# Check for Python
$PythonCmd = Get-Command python -ErrorAction SilentlyContinue
if (-not $PythonCmd) {
    $PythonCmd = Get-Command python3 -ErrorAction SilentlyContinue
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

Write-Host "[WARNING] Neither Python nor Node.js found. Opening index.html directly in browser..." -ForegroundColor Yellow
Start-Process "index.html"
