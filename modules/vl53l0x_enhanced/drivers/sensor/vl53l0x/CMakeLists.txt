zephyr_library()

# Enhanced driver
zephyr_library_sources(vl53l0x_enhanced.c)

# Platform layer (Zephyr I2C bridge for ST HAL)
zephyr_library_sources(vl53l0x_platform.c)

# ST HAL library sources from the hal_st west module
if(NOT ZEPHYR_HAL_ST_MODULE_DIR)
  # Fallback if variable not set by module system
  set(ZEPHYR_HAL_ST_MODULE_DIR ${ZEPHYR_BASE}/../modules/hal/st)
endif()

set(HAL_VL53L0X_DIR ${ZEPHYR_HAL_ST_MODULE_DIR}/sensor/vl53l0x)

zephyr_library_sources(
    ${HAL_VL53L0X_DIR}/api/core/src/vl53l0x_api.c
    ${HAL_VL53L0X_DIR}/api/core/src/vl53l0x_api_ranging.c
    ${HAL_VL53L0X_DIR}/api/core/src/vl53l0x_api_calibration.c
    ${HAL_VL53L0X_DIR}/api/core/src/vl53l0x_api_core.c
    ${HAL_VL53L0X_DIR}/api/core/src/vl53l0x_api_strings.c
)

# ST HAL headers (vl53l0x_api.h, vl53l0x_def.h, etc.)
zephyr_include_directories(${HAL_VL53L0X_DIR}/api/core/inc)

# Platform headers (vl53l0x_platform.h, vl53l0x_platform_log.h)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
