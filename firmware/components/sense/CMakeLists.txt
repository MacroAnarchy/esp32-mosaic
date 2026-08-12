# Mosaic sense engine — native ESP-IDF component (BLE scan + WiFi report +
# ARP discovery). Ported from the Arduino sense engine (firmware/src).
#
# The public header (include/sense_engine.h) is C-callable so the face
# node's app main (ui/main_ui.c) can drive it. config.h is shared with the
# Arduino env (firmware/include) — same MOSAIC_* knobs, one config file.
idf_component_register(
    SRCS "sense_engine.cpp" "arp_neighbors.cpp"
    INCLUDE_DIRS "include" "${CMAKE_SOURCE_DIR}/include"
    REQUIRES bt esp_wifi esp_netif esp_event esp_http_client nvs_flash esp_timer lwip
)
