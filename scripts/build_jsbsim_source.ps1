# Clone and build JSBSim from source natively on Windows.
#
#   powershell -ExecutionPolicy Bypass -File scripts\build_jsbsim_source.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\build_jsbsim_source.ps1 -Repo <fork url> -Ref <branch>
#
# Needs: Git for Windows, CMake 3.15+, and Visual Studio 2022 (Community or
# "Build Tools") with the "Desktop development with C++" workload.
# Result: external\jsbsim\build\src\Release\JSBSim.exe
#
# The WSL2 route (scripts/build_jsbsim_source.sh inside Ubuntu) is the primary,
# tested path; see docs/source_build.md.
param(
    [string]$Repo = "https://github.com/JSBSim-Team/jsbsim.git",
    [string]$Ref = "v1.3.1"
)
$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $PSScriptRoot)

function Assert-Ok($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed (exit $LASTEXITCODE)" } }

foreach ($tool in @("git", "cmake")) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "'$tool' not found on PATH. See docs/source_build.md for install steps."
    }
}

$src = "external\jsbsim"
$build = "$src\build"
New-Item -ItemType Directory -Force -Path external | Out-Null
if (-not (Test-Path "$src\.git")) {
    Write-Host "==> Cloning $Repo ($Ref) into $src"
    git clone --branch $Ref $Repo $src; Assert-Ok "git clone"
} else {
    Write-Host "==> $src already exists; leaving the checkout as-is"
}

Write-Host "==> Configuring (Visual Studio generator)"
cmake -S $src -B $build -DBUILD_PYTHON_MODULE=OFF -DBUILD_DOCS=OFF; Assert-Ok "cmake configure"

Write-Host "==> Building Release (first build takes several minutes)"
cmake --build $build --config Release --target JSBSim; Assert-Ok "cmake build"

$exe = Resolve-Path "$build\src\Release\JSBSim.exe"
Write-Host "==> Smoke test"
& $exe --version; Assert-Ok "JSBSim --version"
Push-Location $src
& $exe --script=scripts/c1723.xml --end=20 | Out-Null; Assert-Ok "c1723 script"
Pop-Location
Write-Host "==> Done. Executable: $exe"
