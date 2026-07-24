# Remove the MoteInference boot task (elevated PowerShell). Stops and unregisters
# it; leaves the pixi env, model cache, and logs in place.

param([string]$TaskName = "MoteInference")

$ErrorActionPreference = "Stop"
if (Get-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue) {
    Stop-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false
    Write-Host "Removed scheduled task '$TaskName'."
} else {
    Write-Host "No scheduled task '$TaskName' found."
}
