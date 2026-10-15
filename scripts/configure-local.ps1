param(
    [Parameter(Mandatory = $true)]
    [string]$BrokerIp,
    [ValidateSet('WPA2', 'WPA3', 'Mixed')]
    [string]$WifiAuth = 'WPA2'
)

$ErrorActionPreference = 'Stop'
$address = $null
if (-not [Net.IPAddress]::TryParse($BrokerIp, [ref]$address) -or
    $address.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork) {
    throw 'BrokerIp must be an IPv4 address, such as 192.168.1.100.'
}

function ConvertTo-CStringLiteral([string]$Value) {
    if ($Value -match '[\x00-\x1f\x7f]') {
        throw 'Network names and passwords cannot contain control characters.'
    }
    return '"' + $Value.Replace('\', '\\').Replace('"', '\"') + '"'
}

$ssid = Read-Host 'Enter your 2.4 GHz Wi-Fi network name (SSID)'
$ssidBytes = [Text.Encoding]::UTF8.GetByteCount($ssid)
if ($ssidBytes -lt 1 -or $ssidBytes -gt 32) {
    throw 'The Wi-Fi name must be 1 to 32 UTF-8 bytes.'
}

$secret = Read-Host 'Enter the Wi-Fi password (hidden)' -AsSecureString
$secretPointer = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secret)
try {
    $password = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($secretPointer)
} finally {
    [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($secretPointer)
}
if ($password.Length -lt 8 -or $password.Length -gt 63) {
    throw 'The WPA2 password must have 8 to 63 characters.'
}

$configDirectory = Join-Path (Split-Path -Parent $PSScriptRoot) `
    'vendor\mtk3smp-rp2040\config'
$wifiExample = Get-Content -Raw -LiteralPath `
    (Join-Path $configDirectory 'wifi_credentials.example.h')
$mqttExample = Get-Content -Raw -LiteralPath `
    (Join-Path $configDirectory 'mqtt_config.example.h')

$wifi = $wifiExample.Replace('UTK_WIFI_CREDENTIALS_SET   0',
                             'UTK_WIFI_CREDENTIALS_SET   1')
$wifi = $wifi.Replace('"replace-with-ssid"',
                      (ConvertTo-CStringLiteral $ssid))
$wifi = $wifi.Replace('"replace-with-password"',
                      (ConvertTo-CStringLiteral $password))
switch ($WifiAuth) {
    'WPA3' { $authMacro = 'CYW43_AUTH_WPA3_SAE_AES_PSK' }
    'Mixed' { $authMacro = 'CYW43_AUTH_WPA3_WPA2_AES_PSK' }
    default { $authMacro = 'CYW43_AUTH_WPA2_AES_PSK' }
}
$wifi = $wifi.Replace('CYW43_AUTH_WPA2_AES_PSK', $authMacro)
$mqtt = $mqttExample.Replace('AAI18_MQTT_CONFIG_SET 0',
                             'AAI18_MQTT_CONFIG_SET 1')
$mqtt = $mqtt.Replace('"192.168.1.100"',
                      (ConvertTo-CStringLiteral $address.ToString()))

if ($wifi -eq $wifiExample -or $mqtt -eq $mqttExample -or
    $wifi.Contains('"replace-with-ssid"') -or
    $wifi.Contains('"replace-with-password"')) {
    throw 'A configuration example changed; no files were written.'
}

$utf8 = New-Object Text.UTF8Encoding($false)
[IO.File]::WriteAllText((Join-Path $configDirectory 'wifi_credentials.h'),
                        $wifi, $utf8)
[IO.File]::WriteAllText((Join-Path $configDirectory 'mqtt_config.h'),
                        $mqtt, $utf8)
Remove-Variable password
Write-Host 'Local Wi-Fi and MQTT configuration saved in Git-ignored files.'
Write-Host 'Run .\scripts\build.ps1 -Profile mqtt when the broker is ready.'
