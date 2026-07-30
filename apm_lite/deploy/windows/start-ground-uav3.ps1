param(
    [string]$SerialPort = "COM3",
    [int]$SerialBaud = 57600,
    [int]$WebPort = 8000,
    [string]$RtspUrl = "rtsp://192.168.1.109:15544/cam",
    [double]$CameraFps = 15.0,
    [string]$VlmEnvFile = ""
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).ProviderPath
$runtimeRoot = Join-Path $env:USERPROFILE ".aeromind"
$python = Join-Path $runtimeRoot "venv\Scripts\python.exe"
$secretFile = Join-Path $runtimeRoot "uav3.env"

if (-not (Test-Path -LiteralPath $python)) {
    throw "Windows ground virtual environment is missing: $python"
}
if (-not (Test-Path -LiteralPath $secretFile)) {
    throw "UAV3 credential file is missing: $secretFile"
}

$secretLine = [IO.File]::ReadAllText($secretFile).Trim()
$prefix = "AEROMIND_UAV3_TOKEN="
if (-not $secretLine.StartsWith($prefix)) {
    throw "UAV3 credential file has an invalid format"
}

$env:AEROMIND_UAV3_TOKEN = $secretLine.Substring($prefix.Length)
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

& $python -m aeromind_apm_lite.ground.browser.app `
    --mode real_serial `
    --host 127.0.0.1 `
    --port $WebPort `
    --vehicle-id 3 `
    --vehicle-name real-uav3 `
    --serial-port $SerialPort `
    --serial-baud $SerialBaud `
    --frame-calibration-id uav3-local-ned-bench-v1 `
    --secret-env AEROMIND_UAV3_TOKEN `
    --static-dir (Join-Path $repoRoot "web") `
    --rtsp-url $RtspUrl `
    --camera-fps $CameraFps

exit $LASTEXITCODE
