# Mosaic UI node — app component of the [env:ui] ESP-IDF build.
#
# PlatformIO registers the env's src_dir (ui/) as an IDF component via
# EXTRA_COMPONENT_DIRS (see firmware/CMakeLists.txt + scripts/espidf_env.py).
# This file is therefore a component CMakeLists, NOT a project root — the
# project root is firmware/CMakeLists.txt.
#
# Builds the face layer (display_face + glow engine) and the sense engine
# (components/sense) into ONE firmware: BLE sensing + WiFi reporting + face.
# The display layer is M5GFX (LovyanGFX, vendored in ./components/) with the
# CO5300 QSPI AMOLED driven through its PSRAM framebuffer; the old raw
# esp_lcd CO5300 driver (esp_lcd_co5300) is no longer used.
idf_component_register(
    SRCS "main_ui.c" "display_face.cpp" "fx/fx_glow.cpp"
    INCLUDE_DIRS "."
    REQUIRES sense M5GFX
)
