[CmdletBinding()]
param(
    [ValidateSet('all', 'px4', 'apm')]
    [string]$Line = 'all',
    [ValidateSet('all', 'real', 'virtual')]
    [string]$World = 'all',
    [switch]$DryRun,
    [switch]$SkipAirSim,
    [switch]$SkipSITL,
    [string]$AirSimPx4Exe = 'D:\Unreal Project\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe',
    [string]$AirSimApmExe = 'D:\Unreal Project\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe',
    [string]$AirSimPx4Project = 'D:\AirSim\AirSim-1.8.1-windows\Unreal\Environments\Blocks\Blocks.uproject',
    [string]$AirSimApmProject = 'D:\AirSim\AirSim-1.8.1-windows\Unreal\Environments\Blocks\Blocks.uproject',
    [string[]]$AirSimPx4Args = @(),
    [string[]]$AirSimApmArgs = @(),
    [string]$WslDistribution = 'Ubuntu-22.04',
    [string]$RuntimeDirectoryWsl = '',
    [string]$AirSimProfileRoot = (Join-Path $env:LOCALAPPDATA 'Aeromind\parallel-domain\airsim'),
    [ValidateRange(10, 600)]
    [int]$AirSimReadyTimeoutSeconds = 180
)

$ErrorActionPreference = 'Stop'

function Convert-ToWslPath {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Distribution
    )
    $prefix = "\\wsl.localhost\$Distribution\"
    if ($Path.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        return '/' + $Path.Substring($prefix.Length).Replace('\', '/')
    }
    $converted = & wsl.exe -d $Distribution -- wslpath -a $Path
    if ($LASTEXITCODE -ne 0) { throw "Failed to map Windows path into WSL: $Path" }
    return $converted.Trim()
}

$scriptDirectoryWsl = Convert-ToWslPath -Path $PSScriptRoot -Distribution $WslDistribution
$wslScript = "$scriptDirectoryWsl/start_all.sh"
$projectDirectoryWsl = (& wsl.exe -d $WslDistribution -- readlink -f "$scriptDirectoryWsl/../..").Trim()
if ($LASTEXITCODE -ne 0 -or -not $projectDirectoryWsl) {
    throw 'Failed to resolve the parallel-domain project directory in WSL'
}
if (-not $RuntimeDirectoryWsl) {
    $RuntimeDirectoryWsl = "$projectDirectoryWsl/.runtime/parallel-domain-sitl"
}
$wslArgs = @($wslScript, '--line', $Line, '--world', $World)
if ($DryRun -or $SkipSITL) { $wslArgs += '--dry-run' }

if ($SkipSITL) {
    Write-Host "Reusing parallel-domain SITL sessions; refreshing isolated settings ($Line/$World)..."
} else {
    Write-Host "Starting parallel-domain WSL orchestration ($Line/$World)..."
}
& wsl.exe -d $WslDistribution -- env "PD_RUNTIME_DIR=$RuntimeDirectoryWsl" bash @wslArgs
if ($LASTEXITCODE -ne 0) {
    throw "WSL SITL orchestration failed with exit code $LASTEXITCODE"
}

if ($SkipAirSim -or $World -eq 'real') {
    Write-Host 'AirSim launch skipped; selected world does not require it.'
    exit 0
}

$runtimeWindows = "\\wsl.localhost\$WslDistribution" + ($RuntimeDirectoryWsl -replace '/', '\')
$airSimRoot = Join-Path $runtimeWindows 'airsim'
$standardSettings = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'AirSim\settings.json'

function Get-OptionalSha256 {
    param([Parameter(Mandatory = $true)][string]$Path)
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { return $null }
    $stream = [System.IO.File]::OpenRead($Path)
    try {
        $sha256 = [System.Security.Cryptography.SHA256]::Create()
        try {
            $hash = $sha256.ComputeHash($stream)
        } finally {
            $sha256.Dispose()
        }
    } finally {
        $stream.Dispose()
    }
    return ([System.BitConverter]::ToString($hash)).Replace('-', '').ToLowerInvariant()
}

$standardHashBefore = Get-OptionalSha256 -Path $standardSettings

function New-AirSimProfile {
    param(
        [Parameter(Mandatory = $true)][string]$Label,
        [Parameter(Mandatory = $true)][string]$SourceSettings,
        [Parameter(Mandatory = $true)][int]$ExpectedRpcPort
    )
    if (-not (Test-Path -LiteralPath $SourceSettings -PathType Leaf)) {
        throw "Generated AirSim settings not found for $Label`: $SourceSettings"
    }
    $profileRoot = Join-Path $AirSimProfileRoot $Label
    New-Item -ItemType Directory -Path $profileRoot -Force | Out-Null
    $profileSettings = Join-Path $profileRoot 'settings.json'
    Copy-Item -LiteralPath $SourceSettings -Destination $profileSettings -Force
    $payload = Get-Content -LiteralPath $profileSettings -Raw | ConvertFrom-Json
    if ($payload.ApiServerPort -ne $ExpectedRpcPort) {
        throw "$Label profile ApiServerPort must be $ExpectedRpcPort"
    }
    if ((Get-Content -LiteralPath $profileSettings -Raw).Contains('__')) {
        throw "$Label profile still contains an unresolved address placeholder"
    }
    return [pscustomobject]@{
        Label = $Label
        Root = $profileRoot
        Settings = $profileSettings
        SettingsSha256 = Get-OptionalSha256 -Path $profileSettings
        RpcPort = $ExpectedRpcPort
    }
}

function Start-AirSimWorld {
    param(
        [Parameter(Mandatory = $true)][pscustomobject]$Profile,
        [Parameter(Mandatory = $true)][string]$Executable,
        [string]$Project,
        [string[]]$ExtraArguments = @(),
        [Parameter(Mandatory = $true)][int]$WindowX
    )
    if (-not (Test-Path -LiteralPath $Executable -PathType Leaf)) {
        throw "AirSim executable not found for $($Profile.Label): $Executable"
    }
    if ($Project -and -not (Test-Path -LiteralPath $Project -PathType Leaf)) {
        throw "AirSim project not found for $($Profile.Label): $Project"
    }
    $logPath = Join-Path $Profile.Root 'ue.log'
    $arguments = @()
    if ($Project) { $arguments += @($Project, '-game') }
    $arguments += @(
        '-Multiprocess',
        '-NoSplash',
        '-windowed',
        '-ResX=900',
        '-ResY=600',
        "-WinX=$WindowX",
        '-WinY=80',
        "-settings=`"$($Profile.Settings)`"",
        "-AbsLog=`"$logPath`""
    )
    $arguments += $ExtraArguments
    if ($DryRun) {
        Write-Host "DRY-RUN AirSim $($Profile.Label): $Executable $($arguments -join ' ')"
        return [pscustomobject]@{ Profile = $Profile; Process = $null; Arguments = $arguments }
    }
    $process = Start-Process -FilePath $Executable -ArgumentList $arguments `
        -WorkingDirectory (Split-Path -Parent $Executable) -PassThru
    Write-Host "Started AirSim $($Profile.Label), PID $($process.Id), settings $($Profile.Settings)"
    return [pscustomobject]@{ Profile = $Profile; Process = $process; Arguments = $arguments }
}

function Wait-AirSimRpc {
    param([Parameter(Mandatory = $true)][pscustomobject]$Launch)
    if ($DryRun) { return }
    $deadline = [DateTime]::UtcNow.AddSeconds($AirSimReadyTimeoutSeconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        if ($Launch.Process.HasExited) {
            throw "AirSim $($Launch.Profile.Label) exited before RPC became ready"
        }
        $listener = Get-NetTCPConnection -State Listen -LocalPort $Launch.Profile.RpcPort `
            -ErrorAction SilentlyContinue
        if ($listener) {
            Write-Host "AirSim $($Launch.Profile.Label) RPC ready on $($Launch.Profile.RpcPort)"
            return
        }
        Start-Sleep -Seconds 1
        $Launch.Process.Refresh()
    }
    throw "AirSim $($Launch.Profile.Label) RPC port $($Launch.Profile.RpcPort) did not become ready"
}

$launches = @()
if ($Line -in @('all', 'px4')) {
    $px4Profile = New-AirSimProfile -Label 'px4' `
        -SourceSettings (Join-Path $airSimRoot 'px4.settings.json') -ExpectedRpcPort 41451
    $launches += Start-AirSimWorld -Profile $px4Profile -Executable $AirSimPx4Exe `
        -Project $AirSimPx4Project -ExtraArguments $AirSimPx4Args -WindowX 40
}
if ($Line -in @('all', 'apm')) {
    $apmProfile = New-AirSimProfile -Label 'apm' `
        -SourceSettings (Join-Path $airSimRoot 'apm.settings.json') -ExpectedRpcPort 41452
    $launches += Start-AirSimWorld -Profile $apmProfile -Executable $AirSimApmExe `
        -Project $AirSimApmProject -ExtraArguments $AirSimApmArgs -WindowX 980
}

foreach ($launch in $launches) { Wait-AirSimRpc -Launch $launch }

$standardHashAfter = Get-OptionalSha256 -Path $standardSettings
if ($standardHashBefore -ne $standardHashAfter) {
    throw "The standard Documents/AirSim/settings.json changed during isolated launch"
}

if (-not $DryRun) {
    $manifestPath = Join-Path $airSimRoot 'live-processes.json'
    $launchedLabels = @($launches | ForEach-Object { $_.Profile.Label })
    $preservedInstances = @()
    if (Test-Path -LiteralPath $manifestPath -PathType Leaf) {
        try {
            $existingManifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
            $preservedInstances = @(
                $existingManifest.instances | Where-Object { $_.line -notin $launchedLabels }
            )
        } catch {
            Write-Warning "Ignoring unreadable previous AirSim process manifest: $manifestPath"
        }
    }
    $manifest = [ordered]@{
        generated_at_utc = [DateTime]::UtcNow.ToString('o')
        standard_settings_path = $standardSettings
        standard_settings_sha256_before = $standardHashBefore
        standard_settings_sha256_after = $standardHashAfter
        isolated_profile_root = $AirSimProfileRoot
        instances = @($preservedInstances) + @(
            $launches | ForEach-Object {
                [ordered]@{
                    line = $_.Profile.Label
                    pid = $_.Process.Id
                    executable = $_.Process.Path
                    settings_path = $_.Profile.Settings
                    settings_sha256 = $_.Profile.SettingsSha256
                    rpc_port = $_.Profile.RpcPort
                    arguments = $_.Arguments
                }
            }
        )
    }
    $manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $manifestPath -Encoding UTF8
    Write-Host "AirSim isolation manifest: $manifestPath"
}

Write-Host 'SITL sessions are running in WSL tmux; AirSim instances use separate -settings profiles.'
