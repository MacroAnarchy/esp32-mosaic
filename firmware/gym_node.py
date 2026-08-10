#!/usr/bin/env python3
"""
Mosaic gym node — Linux node edition (v0.1).

Runs on the Linux Linux box (or any Linux box with a monitor-mode WiFi
adapter). Captures passive WiFi (probe requests + beacons with RSSI from
radiotap headers) and optionally BLE advertisements. Emits ORB protocol v1
envelopes to the gateway over HTTP, or appends to a local JSONL file when
offline (synced later).

PORTABILITY: the envelope schema is identical to the ESP32 firmware —
the gateway/brain does not care whether a tile is a 3€ C3 or an old phone.
Everything below `emit_scan()` is hardware-specific; the schema is the contract.

Usage (as root — monitor mode needs root):
  python3 linux_node.py --interface wlan1 --node linux-node-01
  python3 linux_node.py --interface wlan1 --node linux-node-01 --ble --interval 30

Requirements:
  apt install python3-scapy   (WiFi capture)
  bluez + hcitool             (BLE capture, optional --ble flag)
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

# --- Config (static for now — tuned the hard way) ---
GATEWAY_URL = os.environ.get("MOSAIC_GATEWAY", "http://192.168.1.10:9000/orb/ingest")
NODE_NAME = "linux-node-01"
RSSI_FLOOR = -90   # dBm: below this, ignore (noise)
BLE_SCAN_SECONDS = 8
POLL_SECONDS = 30   # how often to flush a scan batch


def now_ts():
    return int(time.time() * 1000)


def emit_scan(node, payload):
    """ORB protocol v1 envelope — THE CONTRACT. Same on ESP32 and phone."""
    return {
        "v": 1,
        "node": node,
        "type": "scan",
        "ts": now_ts(),
        "payload": payload,
    }


def http_send(envelope):
    """POST envelope to gateway. Returns True on success."""
    try:
        import urllib.request
        req = urllib.request.Request(
            GATEWAY_URL,
            data=json.dumps(envelope).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status == 200
    except Exception:
        return False


def log_local(envelope, path):
    """Offline fallback — append to JSONL for later sync."""
    with open(path, "a") as f:
        f.write(json.dumps(envelope) + "\n")


# --- WiFi capture (monitor mode, passive) ---
def sniff_wifi(interface, seconds):
    """Capture probe requests + beacons via scapy. Returns list of device dicts.

    Each entry: {mac, rssi, type: 'probe'|'beacon', ssid?, first_seen}
    Dedupes by (mac, type) keeping the strongest RSSI in the window.
    """
    try:
        from scapy.all import conf, sniff
        from scapy.layers.dot11 import Dot11, Dot11Beacon, Dot11ProbeReq, RadioTap
    except ImportError:
        print("[!] scapy not installed: apt install python3-scapy", file=sys.stderr)
        return []

    devices = {}

    def handle(pkt):
        if not pkt.haslayer(Dot11):
            return
        dot11 = pkt[Dot11]
        if dot11.addr2 is None:
            return
        mac = dot11.addr2.lower()
        rssi = None
        if pkt.haslayer(RadioTap):
            try:
                rssi = pkt[RadioTap].dbm_ant_signal
            except Exception:
                rssi = None
        if rssi is None or rssi < RSSI_FLOOR:
            return

        ptype = None
        ssid = None
        if pkt.haslayer(Dot11ProbeReq):
            ptype = "probe"
        elif pkt.haslayer(Dot11Beacon):
            ptype = "beacon"
            try:
                ssid = pkt[Dot11Beacon].info.decode(errors="replace").strip()
            except Exception:
                ssid = None
        if ptype is None:
            return

        key = (mac, ptype)
        cur = devices.get(key)
        if cur is None or rssi > cur["rssi"]:
            d = {"mac": mac, "rssi": rssi, "type": ptype, "first_seen": now_ts()}
            if ssid:
                d["ssid"] = ssid
            devices[key] = d

    try:
        sniff(iface=interface, prn=handle, store=False, timeout=seconds,
              monitor=True)
    except Exception as e:
        print(f"[!] sniff error: {e}", file=sys.stderr)
    return list(devices.values())


# --- BLE capture (optional) ---
def sniff_ble(seconds):
    """hcitool lescan passive — returns list of {mac, name?}."""
    devices = {}
    try:
        # Start scan in background
        proc = subprocess.Popen(
            ["hcitool", "lescan", "--passive", "--duplicates"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        time.sleep(seconds)
        proc.terminate()
        out, _ = proc.communicate(timeout=5)
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 1 and ":" in parts[0]:
                mac = parts[0].lower()
                name = " ".join(parts[1:]).strip() or None
                devices[mac] = {"mac": mac, "type": "ble"}
                if name:
                    devices[mac]["name"] = name
    except Exception as e:
        print(f"[!] ble error: {e}", file=sys.stderr)
    return list(devices.values())


def main():
    ap = argparse.ArgumentParser(description="Mosaic gym node (Linux node)")
    ap.add_argument("--interface", default="wlan1", help="monitor-mode interface")
    ap.add_argument("--node", default=NODE_NAME, help="node name in envelope")
    ap.add_argument("--ble", action="store_true", help="also capture BLE")
    ap.add_argument("--interval", type=int, default=POLL_SECONDS,
                    help="seconds between batches")
    ap.add_argument("--once", action="store_true", help="single batch then exit")
    args = ap.parse_args()

    offline_path = f"/root/mosaic-{args.node}.jsonl"
    offline_buf = []
    online = True

    print(f"[*] Mosaic gym node '{args.node}' on {args.interface}"
          f"{' + BLE' if args.ble else ''}")
    print(f"[*] Gateway: {GATEWAY_URL}")

    while True:
        wifi = sniff_wifi(args.interface, max(5, args.interval // 2))
        ble = sniff_ble(BLE_SCAN_SECONDS) if args.ble else []
        all_devs = wifi + ble
        if not all_devs:
            print(f"[.] {now_ts()} — no devices in window")
        else:
            env = emit_scan(args.node, {"devices": all_devs, "n": len(all_devs)})
            if online and http_send(env):
                print(f"[+] {now_ts()} — sent {len(all_devs)} devices")
            else:
                online = False
                offline_buf.append(env)
                log_local(env, offline_path)
                print(f"[!] offline — logged {len(all_devs)} devices to {offline_path}")

        if args.once:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
