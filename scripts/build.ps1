param(
    [ValidateSet('base', 'wifi', 'mqtt')]
    [string]$Profile = 'base',
    [string]$ArmBin = '',
    [string]$MakeExe = ''
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$sdkDirectory = Join-Path $repoRoot '.tools\pico-sdk-2.2.0'
$buildDirectory = Join-Path $repoRoot 'vendor\mtk3smp-rp2040\build_make'

if (-not (Test-Path -LiteralPath (Join-Path $sdkDirectory 'lib\tinyusb\src\tusb.c'))) {
    throw 'Run .\scripts\setup-sdk.ps1 first.'
}
if ($ArmBin -eq '') {
    $toolchains = Get-ChildItem -LiteralPath `
        (Join-Path $env:USERPROFILE '.pico-sdk\toolchain') `
        -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending
    foreach ($candidate in $toolchains) {
        $candidateBin = Join-Path $candidate.FullName 'bin'
        if (Test-Path (Join-Path $candidateBin 'arm-none-eabi-gcc.exe')) {
            $ArmBin = $candidateBin
            break
        }
    }
}
if (($ArmBin -eq '') -or
    (-not (Test-Path (Join-Path $ArmBin 'arm-none-eabi-gcc.exe')))) {
    throw 'ARM GCC not found. Pass -ArmBin with the compiler bin directory.'
}
if ($MakeExe -eq '') {
    $MakeExe = 'C:\mingw64\bin\mingw32-make.exe'
}
if (-not (Test-Path -LiteralPath $MakeExe)) {
    throw 'GNU Make not found. Pass -MakeExe with mingw32-make.exe.'
}

$hostBin = Split-Path -Parent $MakeExe
$gitUnixBin = 'C:\Program Files\Git\usr\bin'
$env:PATH = "$ArmBin;$hostBin;$gitUnixBin;$env:PATH"

$makeArguments = @(
    '-C', $buildDirectory,
    'PROJECT_APP=1', 'SMP=0', 'CONSOLE=usb_cdc',
    'PICO_SDK_PATH=../../../.tools/pico-sdk-2.2.0', '-j4'
)

switch ($Profile) {
    'base' { $makeArguments += 'WIFI=none' }
    'wifi' { $makeArguments += 'WIFI=cyw43' }
    'mqtt' {
        $configDirectory = Join-Path $repoRoot 'vendor\mtk3smp-rp2040\config'
        if (-not (Test-Path (Join-Path $configDirectory 'wifi_credentials.h')) -or
            -not (Test-Path (Join-Path $configDirectory 'mqtt_config.h'))) {
            throw 'Copy and fill wifi_credentials.example.h and mqtt_config.example.h in vendor\mtk3smp-rp2040\config first.'
        }
        $makeArguments += @('WIFI=cyw43', 'WIFI_JOIN=1',
                            'WIFI_NETIF=1', 'WIFI_DHCP=1', 'WIFI_MQTT=1')
    }
}

& $MakeExe @makeArguments
if ($LASTEXITCODE -ne 0) { throw "Build failed with code $LASTEXITCODE" }
Write-Host "UF2 files are in $buildDirectory"
