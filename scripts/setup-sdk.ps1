param()

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$sdkDirectory = Join-Path $repoRoot '.tools\pico-sdk-2.2.0'

if (-not (Test-Path -LiteralPath $sdkDirectory)) {
    git clone --branch 2.2.0 --depth 1 `
        https://github.com/raspberrypi/pico-sdk.git $sdkDirectory
    if ($LASTEXITCODE -ne 0) { throw 'Pico SDK clone failed.' }
}

Push-Location $sdkDirectory
try {
    git submodule update --init --depth 1 `
        lib/cyw43-driver lib/lwip lib/tinyusb
    if ($LASTEXITCODE -ne 0) { throw 'Pico SDK submodule download failed.' }
} finally {
    Pop-Location
}
Write-Host "Pico SDK 2.2.0 is ready at $sdkDirectory"
