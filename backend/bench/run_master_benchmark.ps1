<#
.SYNOPSIS
    One-command launcher for the master sim benchmark (PyBullet + MuJoCo, all
    solvers / arms / scenarios / metrics). Meant for an unattended full run on a
    fast machine.

.DESCRIPTION
    Finds the sim virtualenv (backend/.venv-sim), sets PYTHONPATH, and runs
    `bench/master_sim_benchmark.py` with output tee'd to a timestamped log under
    backend/results/. Any extra arguments are passed straight through to the
    Python script, so e.g.:

        # full paper-grade run (default: 100 trials × 3 seeds = 300/cell, both engines)
        .\bench\run_master_benchmark.ps1

        # fast smoke to confirm the box is set up
        .\bench\run_master_benchmark.ps1 --quick

        # resume after an interruption; skips cells already in the CSV
        .\bench\run_master_benchmark.ps1 --resume

        # drop the two ~1s homotopy solvers, PyBullet only
        .\bench\run_master_benchmark.ps1 --skip-slow --no-mujoco

    The run is crash-safe (CSV rewritten after every cell) and resumable, so it is
    fine to Ctrl-C and relaunch with --resume.

.NOTES
    Requires backend/.venv-sim with pybullet + mujoco + numpy. If that venv is
    missing (e.g. a fresh machine), recreate it with:

        uv python install 3.12
        uv venv --python 3.12 backend/.venv-sim
        uv pip install --python backend/.venv-sim/Scripts/python.exe pybullet mujoco robot_descriptions numpy
        # PyBullet builds from source on Windows -> needs VS2022 C++ Build Tools.
#>

$ErrorActionPreference = "Stop"

# Resolve backend/ as the parent of this script's folder (bench/).
$BackendDir = Split-Path -Parent $PSScriptRoot
$Venv = Join-Path $BackendDir ".venv-sim\Scripts\python.exe"

if (-not (Test-Path $Venv)) {
    Write-Error "Sim venv not found at $Venv. See the .NOTES block in this script to recreate backend/.venv-sim (needs pybullet + mujoco)."
    exit 1
}

$ResultsDir = Join-Path $BackendDir "results"
if (-not (Test-Path $ResultsDir)) { New-Item -ItemType Directory -Path $ResultsDir | Out-Null }

$Stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$OutStem = Join-Path $ResultsDir "master_sim_$Stamp"
$LogFile = "$OutStem.console.log"

Write-Host "Backend : $BackendDir"
Write-Host "Python  : $Venv"
Write-Host "Output  : $OutStem.{csv,md,manifest.json}"
Write-Host "Log     : $LogFile"
Write-Host "Extra   : $($args -join ' ')"
Write-Host ("-" * 70)

Push-Location $BackendDir
try {
    $env:PYTHONPATH = "."
    # pybullet/mujoco print benign banners to stderr (e.g. pybullet's build-time line on
    # first connect); with $ErrorActionPreference = "Stop" a 2>&1 merge turns those into
    # terminating NativeCommandErrors and kills the whole run. Relax to Continue just for
    # this call so stderr is captured in the log/tee without aborting the sweep.
    $ErrorActionPreference = "Continue"
    # --out is prepended so a user-supplied --out in $args would override it (last wins).
    & $Venv -m bench.master_sim_benchmark --out $OutStem @args 2>&1 | Tee-Object -FilePath $LogFile
    $code = $LASTEXITCODE
    $ErrorActionPreference = "Stop"
}
finally {
    Pop-Location
}

Write-Host ("-" * 70)
Write-Host "Exit code: $code   (log: $LogFile)"
exit $code
