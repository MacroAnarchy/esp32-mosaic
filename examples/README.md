# Examples

Products built on the ESP32-Mosaic stack. The stack itself is a general
passive-sensing capability (BLE + WiFi + CSI + anomaly engine); these docs
show concrete things you can build with it.

| Example | What it does | Status |
|---------|--------------|--------|
| [Home security / intruder detection](home-security.md) | RF baseline of your home; unknown device or CSI field deviation = event | 🔄 core sensing done, anomaly engine next |
| [Pet wellness station](pet-wellness-station.md) | Watches your cat/dog through radio alone: baseline, deviations, early illness signs | 🔄 CSI Phase 1+2 done, wander calibration in progress |

The **pet wellness station is the flagship** — it combines everything the stack
does (presence + movement + CSI + anomaly engine), demonstrating that the
technical pieces compose into a real product.
