# ESP32-Mosaic firmware — ESP-IDF project root (used by the [env:ui]
# PlatformIO env; the Arduino [env:esp32] env does not use CMake).
#
# The app component is the env's src_dir (ui/) — PlatformIO registers it
# via EXTRA_COMPONENT_DIRS. The panel driver is vendored next to the UI
# sources (ui/components/esp_lcd_co5300), so it is added to
# EXTRA_COMPONENT_DIRS here. The sense engine is a project component
# (components/sense) and is picked up automatically from this dir.

cmake_minimum_required(VERSION 3.16.0)

list(APPEND EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/ui/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware)
