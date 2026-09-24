#!/usr/bin/env python3
"""
CSI UDP server (binary protocol v2).

Each datagram from the Receiver is one record:
    'C' | ver(1) | mac[6] | rssi(i8) | noise(i8) | sig_mode(u8) | rate(u8) |
    mcs(u8) | cwb(u8) | channel(u8) | secondary(u8) | len(u16 LE) | data[len] (i8)

Separates streams by source MAC (= which illuminator), warns if the PHY rate
of a MAC ever changes (CSI is only comparable at a constant rate), and writes
one CSV per MAC.

Usage:
    python csi_server.py --port 1201 --out ./csi_logs
"""
import argparse
import socket
import struct
import time
import os
import csv
from collections import defaultdict

HDR = 18  # bytes before the CSI payload


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=1201, help="UDP port (CONTROLLER_PORT)")
    ap.add_argument("--out", default=None, help="dir for per-MAC CSV files")
    args = ap.parse_args()

    if args.out:
        os.makedirs(args.out, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(1.0)
    print(f"[server] listening on {args.host}:{args.port}  (Ctrl+C to stop)", flush=True)

    counts = defaultdict(int)
    writers = {}
    rate_seen = {}          # mac -> (sig_mode, rate) last seen, for consistency check
    first_seen = False
    last_report = time.time()

    HEADER_ROW = ["timestamp", "rssi", "noise", "sig_mode", "rate",
                  "mcs", "cwb", "channel", "secondary", "len"]

    def tick():
        nonlocal last_report
        now = time.time()
        if now - last_report >= 1.0:
            summary = "  ".join(f"{m}:{c}" for m, c in sorted(counts.items()))
            print(f"[{time.strftime('%H:%M:%S')}] pkts/MAC -> {summary or '(none yet)'}",
                  flush=True)
            for f, _ in writers.values():
                f.flush()
            last_report = now

    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                tick()
                continue

            if len(data) < HDR or data[0:1] != b"C" or data[1] != 2:
                continue

            mac = data[2:8]
            rssi = struct.unpack_from("b", data, 8)[0]
            noise = struct.unpack_from("b", data, 9)[0]
            sig_mode = data[10]
            rate = data[11]
            mcs = data[12]
            cwb = data[13]
            ch = data[14]
            sec = data[15]
            length = struct.unpack_from("<H", data, 16)[0]
            payload = data[HDR:HDR + length]
            length = len(payload)                       # actual bytes present
            values = list(struct.unpack_from(f"{length}b", payload)) if length else []
            mac_str = ":".join(f"{b:02X}" for b in mac)

            counts[mac_str] += 1

            # Rate-consistency watchdog: CSI is only comparable at one rate/mode.
            key = (sig_mode, rate)
            if mac_str in rate_seen and rate_seen[mac_str] != key:
                print(f"[warn] {mac_str} PHY changed {rate_seen[mac_str]} -> {key} "
                      f"(CSI not comparable across this change)", flush=True)
            rate_seen[mac_str] = key

            if not first_seen:
                first_seen = True
                print(f"[server] first packet from {addr[0]}  mac={mac_str} "
                      f"sig_mode={sig_mode} rate={rate} mcs={mcs} cwb={cwb} "
                      f"ch={ch} rssi={rssi} noise={noise} len={length}", flush=True)

            if args.out:
                if mac_str not in writers:
                    path = os.path.join(args.out, f"csi_{mac_str.replace(':', '')}.csv")
                    f = open(path, "a", newline="")
                    w = csv.writer(f)
                    w.writerow(HEADER_ROW + [f"csi_{i}" for i in range(length)])
                    writers[mac_str] = (f, w)
                _, w = writers[mac_str]
                w.writerow([time.time(), rssi, noise, sig_mode, rate,
                            mcs, cwb, ch, sec, length, *values])

            tick()

    except KeyboardInterrupt:
        print("\n[server] stopping", flush=True)
    finally:
        for f, _ in writers.values():
            f.flush()
            f.close()
        sock.close()


if __name__ == "__main__":
    main()