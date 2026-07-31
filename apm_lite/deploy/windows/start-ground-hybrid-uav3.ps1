param(
    [string]$SerialPort = "COM3",
    [int]$SerialBaud = 57600,
    [int]$WebPort = 8000,
    [int]$SimVehicleId = 1,
    [int]$SimVehicleCount = 1,
    [string]$SimVehicleIds = "",
    [string]$SimSysIds = "",
    [int]$RealVehicleId = 3,
    [string]$RtspUrl = "rtsp://192.168.1.110:15544/cam",
    [string]$AirSimHost = "127.0.0.1",
    [double]$CameraFps = 15.0,
    [string]$VlmEnvFile = "",
    [switch]$SimOnly
)

$ErrorActionPreference = "Stop"

if ($SimOnly) {
    # Pure simulation fleet: the sim vehicles mirror the real fleet numbering
    # 1:1 (UAV 01..04 -> MAVLink sysid 1..4) and the real bridge is disabled.
    $SimVehicleCount = 4
    $RealVehicleId = 0
    if (-not $SimVehicleIds) {
        $SimVehicleIds = "1,2,3,4"
    }
    if (-not $SimSysIds) {
        $SimSysIds = "1,2,3,4"
    }
}

if ($SimVehicleId -eq $RealVehicleId) {
    throw "Simulation and real vehicle IDs must be different"
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).ProviderPath
$runtimeRoot = Join-Path $env:USERPROFILE ".aeromind"
$python = Join-Path $runtimeRoot "venv\Scripts\python.exe"
$secretFile = Join-Path $runtimeRoot "uav3.env"

if (-not (Test-Path -LiteralPath $python)) {
    throw "Windows ground virtual environment is missing: $python"
}
if (-not $SimOnly -and -not (Test-Path -LiteralPath $secretFile)) {
    throw "UAV3 credential file is missing: $secretFile"
}

& $python -c "import pymavlink" 2>$null
if ($LASTEXITCODE -ne 0) {
    throw "Hybrid mode requires pymavlink in the Windows ground virtual environment"
}

if ($SimOnly) {
    $env:AEROMIND_UAV3_TOKEN = ""
} else {
    $secretLine = [IO.File]::ReadAllText($secretFile).Trim()
    $prefix = "AEROMIND_UAV3_TOKEN="
    if (-not $secretLine.StartsWith($prefix)) {
        throw "UAV3 credential file has an invalid format"
    }
    $env:AEROMIND_UAV3_TOKEN = $secretLine.Substring($prefix.Length)
}
$env:PYTHONPATH = Join-Path $repoRoot "src"

if (-not $VlmEnvFile) {
    $VlmEnvFile = Join-Path $runtimeRoot "vlm.env"
}
if (Test-Path -LiteralPath $VlmEnvFile) {
    $allowedVlmVariables = @(
        "AEROMIND_VLM_API_URL",
        "AEROMIND_VLM_API_KEY",
        "AEROMIND_VLM_MODEL",
        "AEROMIND_LLM_MODEL",
        "AEROMIND_VLM_TIMEOUT_SEC"
    )
    foreach ($line in [IO.File]::ReadAllLines($VlmEnvFile)) {
        $entry = $line.Trim()
        if (-not $entry -or $entry.StartsWith("#")) {
            continue
        }
        $separator = $entry.IndexOf("=")
        if ($separator -lt 1) {
            throw "Invalid VLM environment entry"
        }
        $name = $entry.Substring(0, $separator).Trim()
        $value = $entry.Substring($separator + 1).Trim()
        if ($allowedVlmVariables -notcontains $name) {
            throw "Unsupported VLM environment variable: $name"
        }
        Set-Item -Path "Env:$name" -Value $value
    }
}

if ($SimOnly) {
    & $python -m aeromind_apm_lite.ground.browser.app `
        --mode hybrid `
        --host 127.0.0.1 `
        --port $WebPort `
        --sim-vehicle-id $SimVehicleId `
        --sim-vehicle-count $SimVehicleCount `
        --sim-vehicle-ids $SimVehicleIds `
        --sim-sysids $SimSysIds `
        --sim-vehicle-name "SITL UAV $SimVehicleId" `
        --real-vehicle-id 0 `
        --fcu-endpoint "udpin:0.0.0.0:14550" `
        --serial-port $SerialPort `
        --serial-baud $SerialBaud `
        --frame-calibration-id manual-sim-local-ned-v1 `
        --static-dir (Join-Path $repoRoot "web") `
        --airsim-host $AirSimHost `
        --camera-fps $CameraFps
    exit $LASTEXITCODE
}

& $python -m aeromind_apm_lite.ground.browser.app `
    --mode hybrid `
    --host 127.0.0.1 `
    --port $WebPort `
    --sim-vehicle-id $SimVehicleId `
    --sim-vehicle-count $SimVehicleCount `
    --sim-vehicle-ids $SimVehicleIds `
    --sim-sysids $SimSysIds `
    --sim-vehicle-name "SITL UAV $SimVehicleId" `
    --real-vehicle-id $RealVehicleId `
    --real-vehicle-name "real-uav$RealVehicleId" `
    --fcu-endpoint "udpin:0.0.0.0:14550" `
    --serial-port $SerialPort `
    --serial-baud $SerialBaud `
    --frame-calibration-id uav3-local-ned-bench-v1 `
    --secret-env AEROMIND_UAV3_TOKEN `
    --static-dir (Join-Path $repoRoot "web") `
    --airsim-host $AirSimHost `
    --rtsp-url $RtspUrl `
    --camera-fps $CameraFps

exit $LASTEXITCODE
