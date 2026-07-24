# Register the Mote inference servers to start at boot (Windows Task Scheduler).
#
# Run in an ELEVATED (Administrator) PowerShell. Creates a scheduled task
# "MoteInference" that launches run_inference.ps1 at system startup, whether or
# not a user is logged in, and keeps it running (run_inference.ps1 self-restarts
# the servers; this task restarts the runner itself if it ever stops). CUDA
# compute does not need an interactive desktop, so session-0 startup is fine.
#
# You are prompted for the account to run as — use the machine's own user account
# so the HuggingFace cache and pixi install (both under that profile) are found.
# Task Scheduler stores the password; the servers then survive a reboot with no
# manual step, satisfying the reboot criterion.
#
# Alternative: to run the servers as a true Windows service instead, wrap
# run_inference.ps1 with NSSM (https://nssm.cc) — see docs/inference-server.md.

param(
    [string]$TaskName = "MoteInference",
    [string]$RepoPath = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path,
    [string]$User     = "$env:USERDOMAIN\$env:USERNAME"
)

$ErrorActionPreference = "Stop"
$runner = Join-Path $RepoPath "mote_perception\deploy\windows\run_inference.ps1"
if (-not (Test-Path $runner)) { throw "runner not found: $runner" }

$action = New-ScheduledTaskAction -Execute "powershell.exe" `
    -Argument "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$runner`" -RepoPath `"$RepoPath`""
$trigger = New-ScheduledTaskTrigger -AtStartup
# Keep the runner alive: if it ever exits, restart it; no execution time limit.
$settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries `
    -DontStopIfGoingOnBatteries -RestartCount 999 -RestartInterval (New-TimeSpan -Minutes 1) `
    -ExecutionTimeLimit (New-TimeSpan -Seconds 0) -StartWhenAvailable

$cred = Get-Credential -UserName $User -Message "Password for the account that will run the inference servers"

Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $trigger `
    -Settings $settings -RunLevel Highest `
    -User $cred.UserName -Password $cred.GetNetworkCredential().Password -Force

Start-ScheduledTask -TaskName $TaskName
Write-Host "Registered and started scheduled task '$TaskName'."
Write-Host "Logs: $env:LOCALAPPDATA\mote\logs\inference-<date>.log"
Write-Host "Check from the robot with:  pixi run inference-health --host <this-pc>"
