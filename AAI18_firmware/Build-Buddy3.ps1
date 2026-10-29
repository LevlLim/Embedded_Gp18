param([switch]$Normal, [switch]$BarcodeBench)
if ($Normal -and $BarcodeBench) { throw 'Choose either -Normal or -BarcodeBench.' }
$ErrorActionPreference = 'Stop'
$picoTools = Join-Path $env:USERPROFILE '.pico-sdk'
$cmake = Join-Path $picoTools 'cmake/v4.3.4/bin/cmake.exe'
$ninja = Join-Path $picoTools 'ninja/v1.13.2/ninja.exe'
$compilerBin = Join-Path $picoTools 'toolchain/15_2_Rel1/bin'
$env:PATH = "$compilerBin;$(Split-Path $cmake);$(Split-Path $ninja);$env:PATH"
$preset = if ($BarcodeBench) { 'barcode-bench' } elseif ($Normal) { 'usb' } else { 'test-b3' }
$image = if ($BarcodeBench) { 'mtk3pico_smp1_usb_cdc_wifi_barcode_bench.uf2' } elseif ($Normal) { 'mtk3pico_smp1_usb_cdc_wifi.uf2' } else {
    'mtk3pico_smp1_usb_cdc_wifi_test_b3.uf2'
}
Push-Location -LiteralPath $PSScriptRoot
try {
    & $cmake --preset $preset "-DCMAKE_MAKE_PROGRAM=$ninja" `
        -DAAI18_CONSOLE=usb_cdc -DAAI18_WIFI=cyw43 `
        "-DAAI18_BARCODE_BENCH=$([int][bool]$BarcodeBench)"
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
    & $cmake --build --preset $preset --parallel 4
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
    $outputPath = Join-Path $PSScriptRoot "build/$preset/$image"
    if (!(Test-Path -LiteralPath $outputPath)) { throw 'Expected UF2 is missing.' }
    Write-Output "UF2: $outputPath"
} finally {
    Pop-Location
}
