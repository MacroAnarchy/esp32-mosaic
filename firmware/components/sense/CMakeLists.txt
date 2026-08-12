# Mosaic sense engine — native ESP-IDF component (BLE scan + WiFi report +
# ARP discovery + WiFi CSI motion sensing). Ported from the Arduino sense
# engine (firmware/src).
#
# The public header (include/sense_engine.h) is C-callable so the face
# node's app main (ui/main_ui.c) can drive it. config.h is shared with the
# Arduino env (firmware/include) — same MOSAIC_* knobs, one config file.
# esp_wifi_sensing (Apache-2.0, Espressif) is the CSI motion backend; its
# transitive deps (esp-radar, esp_csi_gain_ctrl, esp_radar_motion_dec)
# are resolved by the IDF component manager (see idf_component.yml +
# dependencies.lock).
idf_component_register(
    SRCS "sense_engine.cpp" "arp_neighbors.cpp" "csi_sensing.cpp"
    INCLUDE_DIRS "include" "${CMAKE_SOURCE_DIR}/include"
    REQUIRES bt esp_wifi esp_netif esp_event esp_http_client nvs_flash esp_timer lwip esp_wifi_sensing
)
