zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADNS5050 src/adns5050.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
