param(
    [Parameter(Mandatory = $true)]
    [string]$ListenIp,
    [string]$MosquittoExe = ''
)

$ErrorActionPreference = 'Stop'
$address = $null
if (-not [Net.IPAddress]::TryParse($ListenIp, [ref]$address) -or
    $address.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
    [Net.IPAddress]::IsLoopback($address)) {
    throw "ListenIp must be this PC's LAN IPv4 address."
}

if ($MosquittoExe -eq '') {
    $installed = 'C:\Program Files\mosquitto\mosquitto.exe'
    if (Test-Path -LiteralPath $installed) {
        $MosquittoExe = $installed
    } else {
        $command = Get-Command mosquitto.exe -ErrorAction SilentlyContinue
        if ($null -eq $command) {
            throw 'Mosquitto is not installed. Install it from https://mosquitto.org/download/ first.'
        }
        $MosquittoExe = $command.Source
    }
}
if (-not (Test-Path -LiteralPath $MosquittoExe)) {
    throw 'MosquittoExe does not point to mosquitto.exe.'
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$toolsDirectory = Join-Path $repoRoot '.tools'
New-Item -ItemType Directory -Path $toolsDirectory -Force | Out-Null
$configPath = Join-Path $toolsDirectory 'mosquitto-bench.conf'
$config = "listener 1883 $($address.ToString())`nallow_anonymous true`npersistence false`n"
[IO.File]::WriteAllText($configPath, $config,
                        (New-Object Text.UTF8Encoding($false)))

Write-Host 'Bench broker listening on' $address.ToString() 'port 1883.'
Write-Host 'Keep this window open during the test. Press Ctrl+C to stop it.'
Write-Host 'Use only with the motor battery disconnected.'
& $MosquittoExe -c $configPath -v
if ($LASTEXITCODE -ne 0) {
    throw "Mosquitto stopped with code $LASTEXITCODE. Check the port and Windows Firewall."
}
