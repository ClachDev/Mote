# One-shot setup for the Mote inference PC (Windows + NVIDIA GPU).
#
# Run once in a normal (non-admin) PowerShell from anywhere. It:
#   1. installs pixi if it isn't already on the machine,
#   2. solves + installs the win-64 CUDA inference env (torch cu128 wheel),
#   3. verifies the GPU is visible to torch,
#   4. pre-fetches the depth + detect model weights into the HF cache.
#
# After this, install_service.ps1 makes the servers start at boot. The whole
# thing is idempotent — safe to re-run after a repo update or a driver change.

param(
    [string]$RepoPath = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
)

$ErrorActionPreference = "Stop"
$pixi = Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"

if (-not (Test-Path $pixi)) {
    Write-Host "Installing pixi ..."
    powershell -ExecutionPolicy Bypass -Command "iwr -useb https://pixi.sh/install.ps1 | iex"
}
if (-not (Test-Path $pixi)) {
    throw "pixi install did not produce $pixi — install manually from https://pixi.sh and re-run."
}

Set-Location $RepoPath
Write-Host "Solving + installing the inference-cuda env (first run downloads torch; be patient) ..."
& $pixi install -e inference-cuda

Write-Host "`nGPU check:"
& $pixi run -e inference-cuda python -c "import torch; print('torch', torch.__version__); print('cuda_available', torch.cuda.is_available()); print('device', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NONE')"

Write-Host "`nPre-fetching model weights ..."
& $pixi run inference-prefetch-cuda

Write-Host "`nSetup complete. Start the servers now with:  pixi run inference-cuda"
Write-Host "Install boot auto-start with:  .\mote_perception\deploy\windows\install_service.ps1 (as Administrator)"
