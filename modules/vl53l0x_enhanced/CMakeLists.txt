if(CONFIG_VL53L0X_ENHANCED)
  add_subdirectory(drivers/sensor/vl53l0x)

  # Make public header available to the application
  zephyr_include_directories(include)
endif()
