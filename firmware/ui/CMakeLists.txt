# Mosaic UI node — app component of the [env:ui] ESP-IDF build.
#
# PlatformIO registers the env's src_dir (ui/) as an IDF component via
# EXTRA_COMPONENT_DIRS (see firmware/CMakeLists.txt + scripts/espidf_env.py).
# This file is therefore a component CMakeLists, NOT a project root — the
# project root is firmware/CMakeLists.txt.
#
# Builds the face layer (display_face + glow engine) and the sense engine
# (components/sense) into ONE firmware: BLE sensing + WiFi reporting + face.
# The vendored CO5300 panel driver lives in ./components/ and is discovered
# through firmware/CMakeLists.txt's EXTRA_COMPONENT_DIRS.
idf_component_register(
    SRCS "main_ui.c" "display_face.cpp" "fx/fx_glow.cpp"
         "orb_pmu.cpp" "port_axp2101.cpp"
    INCLUDE_DIRS "." "fx"
    REQUIRES sense esp_lcd_co5300 esp_lcd driver esp_timer
)
