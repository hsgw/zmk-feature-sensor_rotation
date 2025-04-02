zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SENSOR_ROTATION src/sensor_rotation.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)