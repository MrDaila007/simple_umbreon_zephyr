cmake_minimum_required(VERSION 3.20.0)

# USB CDC console always enabled for this build
list(APPEND EXTRA_CONF_FILE        ${CMAKE_CURRENT_SOURCE_DIR}/usb_console.conf)
list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/usb_console.overlay)

# Out-of-tree VL53L0X enhanced driver
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/vl53l0x_enhanced
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(simple_umbreon_zephyr)

target_sources(app PRIVATE
    src/main.c
    src/settings.c
    src/car.c
    src/tachometer.c
    src/sensors.c
    src/battery.c
    src/wifi_cmd.c
    src/control.c
)
