param(
    [string]$Compiler = 'gcc',
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'host-output')
)

$ErrorActionPreference = 'Stop'
$appRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$stubRoot = (Resolve-Path (Join-Path $appRoot '../testing/stub')).Path
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$outputRoot = (Resolve-Path $OutputDirectory).Path
$flags = @('-std=c11', '-Wall', '-Wextra', '-Wconversion',
    '-Wsign-conversion', '-Wshadow', '-Wpointer-arith', '-Wcast-qual',
    '-Wstrict-prototypes', '-Werror', '-O1', '-g', '-I', $stubRoot,
    '-I', $appRoot)

$suites = @(
    @{ Name = 'test_buddy3'; Sources = @(
        (Join-Path $PSScriptRoot 'test_buddy3.c'),
        (Join-Path $appRoot 'subsystem3/barcode_mapping.c'),
        (Join-Path $appRoot 'msg_defs.c')) },
    @{ Name = 'test_mapping'; Sources = @(
        (Join-Path $PSScriptRoot 'test_mapping.c')) },
    @{ Name = 'test_adc'; Sources = @(
        (Join-Path $PSScriptRoot 'test_adc.c')) },
    @{ Name = 'test_startup'; Sources = @(
        (Join-Path $PSScriptRoot 'test_startup.c')) },
    @{ Name = 'test_telemetry'; Sources = @(
        (Join-Path $PSScriptRoot 'test_telemetry.c'),
        (Join-Path $appRoot 'msg_defs.c')) }
)

foreach ($suite in $suites) {
    $executable = Join-Path $outputRoot ($suite.Name + '.exe')
    & $Compiler @flags @($suite.Sources) -o $executable
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($suite.Name)" }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw "Tests failed: $($suite.Name)" }
}
