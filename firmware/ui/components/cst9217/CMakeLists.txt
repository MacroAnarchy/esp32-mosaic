# Vendored CST9217 capacitive touch driver (Waveshare, Apache-2.0).
# Copy of the verified driver from orb-csi-test (components/cst9217):
# drives the Waveshare ESP32-S3-Touch-AMOLED-1.75C touch controller over
# I2C through the esp_lcd_touch API. `esp_lcd_touch` itself is fetched
# from the component registry (see idf_component.yml).
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(PRIV_REQ esp_driver_gpio esp_driver_i2c)
else()
    set(PRIV_REQ driver)
endif()

idf_component_register(SRCS "esp_lcd_touch_cst9217.c" INCLUDE_DIRS "include" REQUIRES "esp_lcd" PRIV_REQUIRES ${PRIV_REQ})
