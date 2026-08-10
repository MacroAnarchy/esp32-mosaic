"""
Pre-build config guard for ESP32-Mosaic firmware.

Refuses to compile/flash if include/config.h still contains the EXAMPLE
placeholder values. Background (2026-08-10 incident): the BLE/WiFi port
overwrote node-01's working config.h with the example copy, the new firmware
was flashed, and the node could not join "YOUR_WIFI_SSID" — a dead node,
silent pipeline for ~20 min. A node that can't rejoin WiFi is dead; this
guard makes that failure impossible to build.

Wire-up (platformio.ini):
    extra_scripts = pre:scripts/check_config.py

The check is deliberately strict on the example values and permissive
otherwise: any real SSID/password/node name passes.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else None


def _find_config_h():
    """Locate include/config.h regardless of where PlatformIO exec's us from.

    Candidates, in order: explicit env override (tests), __file__-relative
    (CLI), then walking up the tree from CWD looking for a project layout
    (include/config.h with a platformio.ini nearby).
    """
    if os.environ.get("MOSAIC_CONFIG_H"):
        return os.environ["MOSAIC_CONFIG_H"]
    candidates = []
    if HERE:
        candidates.append(os.path.join(HERE, "..", "include", "config.h"))
    d = os.getcwd()
    for _ in range(6):
        candidates.append(os.path.join(d, "include", "config.h"))
        candidates.append(os.path.join(d, "firmware", "include", "config.h"))
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0] if candidates else "include/config.h"


CONFIG_H = _find_config_h()

PLACEHOLDER_MARKERS = ("YOUR_", "your_", "CHANGE_ME", "changeme", "<", ">")

# Example defaults that are NOT placeholders but still brick a node
# (mosaic-01 is config.example.h's node name; every node must be unique).
BAD_NODE_NAMES = {"mosaic-01", "Mosaic-01", "MOSAIC-01"}


def _value(line):
    m = re.search(r'"([^"]*)"', line)
    return m.group(1) if m else ""


def main():
    if not os.path.exists(CONFIG_H):
        print("check_config: include/config.h missing — copy config.example.h "
              "and fill in real values before building.")
        sys.exit(1)

    problems = []
    values = {}
    with open(CONFIG_H) as f:
        for line in f:
            line = line.strip()
            for key in ("MOSAIC_WIFI_SSID", "MOSAIC_WIFI_PASSWORD",
                        "MOSAIC_GATEWAY_HOST", "MOSAIC_NODE_NAME"):
                if line.startswith("#define " + key):
                    values[key] = _value(line)

    ssid = values.get("MOSAIC_WIFI_SSID", "")
    psk = values.get("MOSAIC_WIFI_PASSWORD", "")
    host = values.get("MOSAIC_GATEWAY_HOST", "")
    node = values.get("MOSAIC_NODE_NAME", "")

    if not ssid:
        problems.append("MOSAIC_WIFI_SSID is missing")
    elif any(m in ssid for m in PLACEHOLDER_MARKERS):
        problems.append(f"MOSAIC_WIFI_SSID is a placeholder ({ssid!r}) — "
                        "set your real WiFi SSID in include/config.h")

    if not psk:
        problems.append("MOSAIC_WIFI_PASSWORD is missing")
    elif any(m in psk for m in PLACEHOLDER_MARKERS):
        problems.append("MOSAIC_WIFI_PASSWORD is a placeholder — set the real "
                        "WiFi password in include/config.h")

    if not host:
        problems.append("MOSAIC_GATEWAY_HOST is missing")
    elif any(m in host for m in PLACEHOLDER_MARKERS):
        problems.append(f"MOSAIC_GATEWAY_HOST is a placeholder ({host!r})")

    if not node:
        problems.append("MOSAIC_NODE_NAME is missing")
    elif node in BAD_NODE_NAMES or any(m in node for m in PLACEHOLDER_MARKERS):
        problems.append(f"MOSAIC_NODE_NAME is the example default ({node!r}) — "
                        "give this node a unique name (e.g. node-01, orb, sentinel-1)")

    if problems:
        print("check_config: BUILD BLOCKED — include/config.h has unconfigured values.")
        for p in problems:
            print("  - " + p)
        print("Fix include/config.h first (see include/config.example.h).")
        sys.exit(1)

    print(f"check_config: config.h OK (node={node}, ssid={ssid}, gateway={host})")


# PlatformIO exec's pre-scripts as an import (__name__ != "__main__"), so the
# check must run at module level. sys.exit() propagates and aborts the build.
main()
