param(
    [string]$Destination = "pi-py39-wheelhouse"
)

$ErrorActionPreference = "Stop"
$destinationPath = [System.IO.Path]::GetFullPath($Destination)
$requirementsPath = Join-Path $PSScriptRoot "pi-py39-requirements.txt"

New-Item -ItemType Directory -Force -Path $destinationPath | Out-Null

python -m pip download `
    --dest $destinationPath `
    --only-binary=:all: `
    "pip==25.3"

python -m pip download `
    --requirement $requirementsPath `
    --dest $destinationPath `
    --only-binary=:all: `
    --platform manylinux2014_aarch64 `
    --python-version 39 `
    --implementation cp `
    --abi cp39

Write-Host "Raspberry Pi wheelhouse ready: $destinationPath"
