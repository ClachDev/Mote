# Boot-launched runner for the Mote inference servers on the Windows gaming PC.
#
# Runs `pixi run inference-cuda` (both depth + detect servers, bound to 0.0.0.0)
# in a supervise loop: if the pixi process exits — because a server crashed and
# the Python supervisor tore the rest down so the failure is visible — this waits
# a few seconds and starts it again, so the machine self-heals without waiting on
# Task Scheduler's restart policy. All output is tee'd to a dated log file.
#
# install_service.ps1 registers this to run at boot; you can also run it by hand
# in a terminal to watch the servers live. Ctrl+C stops the loop.
#
# Params let a PC rebuild point at a different checkout / pixi / log dir without
# editing the script.

param(
    [string]$RepoPath = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path,
    [string]$Pixi     = (Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"),
    [string]$LogDir   = (Join-Path $env:LOCALAPPDATA "mote\logs"),
    [int]$RestartDelaySeconds = 5
)

$ErrorActionPreference = "Stop"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$log = Join-Path $LogDir ("inference-{0}.log" -f (Get-Date -Format "yyyyMMdd"))

function Write-Log($msg) {
    $line = "{0}  {1}" -f (Get-Date -Format "s"), $msg
    $line | Tee-Object -FilePath $log -Append
}

if (-not (Test-Path $Pixi)) {
    Write-Log "pixi not found at $Pixi — run setup.ps1 first. Exiting."
    exit 1
}

Write-Log "runner up: repo=$RepoPath pixi=$Pixi log=$log"
Set-Location $RepoPath

while ($true) {
    Write-Log "starting: pixi run inference-cuda"
    # Stream the servers' stdout/stderr into the same log.
    & $Pixi "run" "inference-cuda" 2>&1 | Tee-Object -FilePath $log -Append
    $code = $LASTEXITCODE
    Write-Log "inference-cuda exited ($code); restarting in ${RestartDelaySeconds}s"
    Start-Sleep -Seconds $RestartDelaySeconds
}
