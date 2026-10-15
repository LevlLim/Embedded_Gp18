#!/usr/bin/env python3
"""
Echo server for the micro T-Kernel 3.0 RP2040 networking profiles.

The Pico W is the client: it sends, and expects the same bytes back. This
script is the other end. Run it on any Linux host on the same network --
a Raspberry Pi Zero 2 W is fine -- before flashing a WIFI_UDP or WIFI_TCP
build.

    UDP  port 7007   64 packets x 48 bytes, stop-and-wait
    TCP  port 7008   connect, echo, close

Both are plain byte-for-byte echo: nothing here parses the payload, so the
same script serves every profile.

Usage:
    ./linux_echo.py                 # both UDP and TCP, default ports
    ./linux_echo.py --udp-port 7007 --tcp-port 7008
    ./linux_echo.py --udp-only
    ./linux_echo.py --quiet         # counts only, no per-packet lines

Put this host's address in the Pico's config before building:

    cp config/udp_test_config.example.h config/udp_test_config.h
    # set UTK_UDP_ECHO_ADDRESS to the address printed below
"""

import argparse
import socket
import socketserver
import sys
import threading


def local_addresses():
    """Best-effort list of this host's non-loopback IPv4 addresses."""
    found = []
    try:
        # No traffic is sent; this just asks the routing table which source
        # address would be used to reach the outside world.
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.0.2.1", 9))          # TEST-NET-1, guaranteed unrouted
        found.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       socket.AF_INET):
            addr = info[4][0]
            if not addr.startswith("127.") and addr not in found:
                found.append(addr)
    except OSError:
        pass
    return found


class UDPEcho(socketserver.BaseRequestHandler):
    count = 0
    total = 0

    def handle(self):
        data, sock = self.request
        sock.sendto(data, self.client_address)
        UDPEcho.count += 1
        UDPEcho.total += len(data)
        if not self.server.quiet:
            print(f"udp  packet {UDPEcho.count}: {len(data)} bytes "
                  f"<-> {self.client_address[0]}:{self.client_address[1]}",
                  flush=True)


class TCPEcho(socketserver.BaseRequestHandler):
    sessions = 0
    total = 0

    def handle(self):
        TCPEcho.sessions += 1
        n = TCPEcho.sessions
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        print(f"tcp  session {n}: connected from {peer}", flush=True)
        bytes_this = 0
        while True:
            data = self.request.recv(4096)
            if not data:
                break
            self.request.sendall(data)
            bytes_this += len(data)
            TCPEcho.total += len(data)
            if not self.server.quiet:
                print(f"tcp  session {n}: echoed {len(data)} bytes",
                      flush=True)
        print(f"tcp  session {n}: closed after {bytes_this} bytes", flush=True)


class ReusingUDPServer(socketserver.UDPServer):
    allow_reuse_address = True


class ReusingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser(
        description="UDP/TCP echo server for the RP2040 uT-Kernel port")
    ap.add_argument("--bind", default="0.0.0.0",
                    help="address to listen on (default: all interfaces)")
    ap.add_argument("--udp-port", type=int, default=7007,
                    help="must match UTK_UDP_ECHO_PORT (default: 7007)")
    ap.add_argument("--tcp-port", type=int, default=7008,
                    help="must match UTK_TCP_ECHO_PORT (default: 7008)")
    ap.add_argument("--udp-only", action="store_true")
    ap.add_argument("--tcp-only", action="store_true")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress per-packet lines")
    args = ap.parse_args()

    servers = []
    if not args.tcp_only:
        u = ReusingUDPServer((args.bind, args.udp_port), UDPEcho)
        u.quiet = args.quiet
        servers.append(("UDP", args.udp_port, u))
    if not args.udp_only:
        t = ReusingTCPServer((args.bind, args.tcp_port), TCPEcho)
        t.quiet = args.quiet
        servers.append(("TCP", args.tcp_port, t))

    addrs = local_addresses()
    print("micro T-Kernel RP2040 echo server")
    if addrs:
        print(f"  this host: {', '.join(addrs)}")
        print(f"  put that address in config/udp_test_config.h "
              f"(UTK_UDP_ECHO_ADDRESS)")
    for proto, port, _ in servers:
        print(f"  listening {proto} on {args.bind}:{port}")
    print("  Ctrl+C to stop\n", flush=True)

    threads = []
    for _, _, srv in servers:
        th = threading.Thread(target=srv.serve_forever, daemon=True)
        th.start()
        threads.append(th)

    try:
        while True:
            for th in threads:
                th.join(0.5)
    except KeyboardInterrupt:
        print("\nstopping", flush=True)
    finally:
        for _, _, srv in servers:
            srv.shutdown()
            srv.server_close()
        print(f"udp: {UDPEcho.count} packets, {UDPEcho.total} bytes")
        print(f"tcp: {TCPEcho.sessions} sessions, {TCPEcho.total} bytes")


if __name__ == "__main__":
    sys.exit(main())
