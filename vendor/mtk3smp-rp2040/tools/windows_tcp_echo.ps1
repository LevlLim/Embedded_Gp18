param(
    [ValidateRange(1, 65535)]
    [int]$Port = 7008,

    [string]$ListenAddress = "0.0.0.0"
)

$LocalAddress = [System.Net.IPAddress]::Parse($ListenAddress)
$Listener = [System.Net.Sockets.TcpListener]::new($LocalAddress, $Port)
$Sessions = 0

try {
    $Listener.Start()
    Write-Host "TCP echo server listening on ${ListenAddress}:$Port"
    Write-Host "Keep this window open while the Pico W qualification runs."
    Write-Host "Press Ctrl+C to stop."

    while ($true) {
        if (-not $Listener.Pending()) {
            Start-Sleep -Milliseconds 200
            continue
        }

        $Client = $Listener.AcceptTcpClient()
        $Sessions++
        $Peer = $Client.Client.RemoteEndPoint
        Write-Host ("session {0}: connected from {1}" -f $Sessions, $Peer)

        $Bytes = 0
        $Records = 0
        try {
            $Client.NoDelay = $true
            $Stream = $Client.GetStream()
            $Stream.ReadTimeout = 30000
            $Buffer = [byte[]]::new(2048)

            while ($true) {
                $Read = $Stream.Read($Buffer, 0, $Buffer.Length)
                if ($Read -le 0) { break }   # client half-closed: its FIN arrived
                $Stream.Write($Buffer, 0, $Read)
                $Stream.Flush()
                $Bytes += $Read
                $Records++
                Write-Host ("session {0}: echoed {1} bytes (total {2})" -f `
                    $Sessions, $Read, $Bytes)
            }

            # Answer the client's FIN with our own so the close is orderly in
            # both directions; the Pico W gate waits for exactly this.
            $Client.Client.Shutdown(
                [System.Net.Sockets.SocketShutdown]::Send)
            Write-Host ("session {0}: peer closed, sent FIN after {1} reads, {2} bytes" -f `
                $Sessions, $Records, $Bytes)
        }
        catch [System.IO.IOException] {
            Write-Host ("session {0}: read/write failed after {1} bytes: {2}" -f `
                $Sessions, $Bytes, $_.Exception.Message)
        }
        finally {
            $Client.Close()
        }
    }
}
finally {
    $Listener.Stop()
    Write-Host "TCP echo server stopped: $Sessions sessions"
}
