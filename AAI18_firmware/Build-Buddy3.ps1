param([switch]$Normal)
$ErrorActionPreference = 'Stop'
$picoTools = Join-Path $env:USERPROFILE '.pico-sdk'
$cmake = Join-Path $picoTools 'cmake/v4.3.4/bin/cmake.exe'
$ninja = Join-Path $picoTools 'ninja/v1.13.2/ninja.exe'
$compilerBin = Join-Path $picoTools 'toolchain/15_2_Rel1/bin'
$env:PATH = "$compilerBin;$(Split-Path $cmake);$(Split-Path $ninja);$env:PATH"
$preset = if ($Normal) { 'usb' } else { 'test-b3' }
$image = if ($Normal) { 'mtk3pico_smp1_usb_cdc_wifi.uf2' } else {
    'mtk3pico_smp1_usb_cdc_wifi_test_b3.uf2'
}
Push-Location -LiteralPath $PSScriptRoot
try {
    & $cmake --preset $preset "-DCMAKE_MAKE_PROGRAM=$ninja" `
        -DAAI18_CONSOLE=usb_cdc -DAAI18_WIFI=cyw43
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
    & $cmake --build --preset $preset --parallel 4
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
    $outputPath = Join-Path $PSScriptRoot "build/$preset/$image"
    if (!(Test-Path -LiteralPath $outputPath)) { throw 'Expected UF2 is missing.' }
    Write-Output "UF2: $outputPath"
} finally {
    Pop-Location
}
