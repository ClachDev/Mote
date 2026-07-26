# Update the Mote inference PC to the current branch head.
#
# The robot is deployed by rsync (`pixi run sync`), but this machine is a git
# clone — it has no ROS and only needs the repo plus the inference-cuda env, so it
# updates by pulling rather than being pushed to. That also keeps the reported
# revision meaningful: `pixi run inference-health` on the robot prints the
# server's `git describe`, so you can see at a glance whether this PC is stale.
#
# Run in a normal PowerShell on the inference PC:
#     .\mote_perception\deploy\windows\update.ps1
#
# Steps: stop the servers, pull, re-solve the env (a changed pixi.lock is picked
# up here), re-fetch any new models, restart. Stopping first avoids updating
# files under running servers; the boot task restarts them at the end.
#
# -Branch defaults to whatever the checkout is on, so this follows a feature
# branch during bring-up and main afterwards without editing anything.

param(
    [string]$RepoPath = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path,
    [string]$Pixi     = (Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"),
    [string]$Branch   = "",
    [string]$TaskName = "MoteInference"
)

$ErrorActionPreference = "Stop"
Set-Location $RepoPath

$task = Get-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
if ($task) {
    Write-Host "Stopping $TaskName ..."
    Stop-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 2
}

Write-Host "Before: $(git describe --always --dirty --tags)"

if ($Branch) { git checkout $Branch }
git pull --ff-only

Write-Host "Re-solving the inference-cuda env ..."
& $Pixi install -e inference-cuda

Write-Host "Fetching any new models ..."
& $Pixi run inference-prefetch-cuda

Write-Host "After:  $(git describe --always --dirty --tags)"

if ($task) {
    Write-Host "Restarting $TaskName ..."
    Start-ScheduledTask -TaskName $TaskName
} else {
    Write-Host "No boot task installed; start the servers with:  pixi run inference-cuda"
}

Write-Host "`nConfirm from the robot:  pixi run inference-health --host $env:COMPUTERNAME"
