# One-time setup on Windows (PowerShell).  Run from the repo root:
#
#   powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
#
# Creates .venv, installs JSBSim + the lesson helpers, and runs the install check.
Set-Location (Split-Path -Parent $PSScriptRoot)

# Prefer the Python launcher ("py") that python.org installs; fall back to "python".
$py = $null
foreach ($cand in @("py -3.12", "py -3.13", "py -3.11", "py -3", "python")) {
    $exe, $arg = $cand.Split(" ", 2)
    if (Get-Command $exe -ErrorAction SilentlyContinue) {
        try {
            if ($arg) { & $exe $arg --version *> $null } else { & $exe --version *> $null }
            if ($LASTEXITCODE -eq 0) { $py = $cand; break }
        } catch { }
    }
}
$ErrorActionPreference = "Stop"
if (-not $py) {
    Write-Error "Python 3.10+ not found. Install Python 3.12 from https://www.python.org/downloads/windows/ (tick 'Add python.exe to PATH')."
}
Write-Host "Using: $py"

if (-not (Test-Path ".venv")) {
    $exe, $arg = $py.Split(" ", 2)
    if ($arg) { & $exe $arg -m venv .venv } else { & $exe -m venv .venv }
}

$vpy = ".\.venv\Scripts\python.exe"
function Invoke-Step([string[]]$cmdArgs) {
    & $vpy @cmdArgs
    if ($LASTEXITCODE -ne 0) { Write-Error "Failed: python $($cmdArgs -join ' ')" }
}
Invoke-Step @("-m", "pip", "install", "--upgrade", "pip")
Invoke-Step @("-m", "pip", "install", "-r", "requirements.txt")
Invoke-Step @("-m", "pip", "install", "-e", ".")
Invoke-Step @("scripts\verify_install.py")

Write-Host ""
Write-Host "Next time, activate the environment with:  .\.venv\Scripts\Activate.ps1"
