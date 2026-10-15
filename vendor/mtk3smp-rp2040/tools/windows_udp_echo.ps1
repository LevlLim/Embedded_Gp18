param(
    [ValidateRange(1, 65535)]
    [int]$Port = 7007,

    [string]$ListenAddress = "0.0.0.0"
)

$LocalAddress = [System.Net.IPAddress]::Parse($ListenAddress)
$LocalEndpoint = [System.Net.IPEndPoint]::new($LocalAddress, $Port)
$RemoteEndpoint = [System.Net.IPEndPoint]::new(
    [System.Net.IPAddress]::Any, 0)
$Udp = [System.Net.Sockets.UdpClient]::new()
$Packets = 0
$Bytes = 0

try {
    $Udp.Client.Bind($LocalEndpoint)
    $Udp.Client.ReceiveTimeout = 1000
    Write-Host "UDP echo server listening on ${ListenAddress}:$Port"
    Write-Host "Keep this window open while the Pico W qualification runs."
    Write-Host "Press Ctrl+C to stop."

    while ($true) {
        try {
            $Payload = $Udp.Receive([ref]$RemoteEndpoint)
            $Sent = $Udp.Send($Payload, $Payload.Length, $RemoteEndpoint)
            $Packets++
            $Bytes += $Sent
            Write-Host ("echoed packet {0}: {1} bytes to {2}" -f `
                $Packets, $Sent, $RemoteEndpoint)
        }
        catch [System.Net.Sockets.SocketException] {
            $Code = $_.Exception.SocketErrorCode
            if ($Code -ne [System.Net.Sockets.SocketError]::TimedOut -and
                $Code -ne [System.Net.Sockets.SocketError]::WouldBlock) {
                throw
            }
        }
    }
}
finally {
    $Udp.Dispose()
    Write-Host "UDP echo server stopped: $Packets packets, $Bytes bytes"
}
