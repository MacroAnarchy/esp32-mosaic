# THIRD-PARTY NOTICES — esp32-mosaic

This project builds on the work of others. All components are redistributed
under their original licenses. Our own code is MIT (see LICENSE).

## Components vendored / managed in-tree

| Component | License | Copyright / Source |
|-----------|---------|--------------------|
| M5GFX (LovyanGFX) 0.2.19 | MIT | © 2021 M5Stack — https://github.com/m5stack/M5GFX |
| esp-radar (esp_wifi_sensing radar FSM) | Apache-2.0 | Espressif Systems — https://components.espressif.com/components/espressif/esp-radar |
| esp_wifi_sensing | Apache-2.0 | Espressif Systems — https://components.espressif.com/components/espressif/esp_wifi_sensing |
| esp_radar_motion_dec | Apache-2.0 | Espressif Systems — https://components.espressif.com/components/espressif/esp_radar_motion_dec |
| esp_csi_gain_ctrl | Apache-2.0 | Espressif Systems — https://components.espressif.com/components/espressif/esp_csi_gain_ctrl |
| cst9217 touch driver (vendored) | Apache-2.0 | Espressif Systems esp_lcd_touch family — https://github.com/espressif/esp-lcd-touch |
| esp_lcd_touch | Apache-2.0 | Espressif Systems — https://github.com/espressif/esp-lcd-touch |

## Reference projects (code or patterns adapted, not vendored)

| Project | License | Why it matters |
|---------|---------|----------------|
| M5Stack M5StopWatch-Flux | MIT | Display HAL (Panel_CO5300 over M5GFX), init sequence, framebuffer pattern — display_face.cpp is explicitly ported from its hal_display.cpp |
| Espressif esp-csi | Apache-2.0 | Official CSI toolkit: csi_recv/csi_send/csi_recv_router, esp-radar examples, data parsing tools |
| RuView (ruvnet) | see repo | Research reference: through-wall presence, breathing/HR pipelines, HA integration |
| DeFall (Hu et al., IEEE INFOCOM 2020) | paper | Burst+stillness fall-detection algorithm basis (published, not patented) |
| ESP32Marauder | GPL-3.0 (reference only) | OUI/vendor identification + packet-capture patterns; no code vendored |

## License compatibility note

MIT root + Apache-2.0 components are compatible (both permissive). Apache-2.0
requires preserving this notice file when the covered components are
redistributed — it is shipped verbatim with the project.

_Generated 2026-08-13. Update when new components land._
