# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT

# Add this to your ZMK module's drivers/sensor/CMakeLists.txt

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_INPUT_ADNS5050 adns5050.c)

# If creating as a standalone module, use this CMakeLists.txt structure:
# 
# project root/
# ├── CMakeLists.txt (this file)
# ├── Kconfig
# ├── drivers/
# │   └── sensor/
# │       ├── adns5050/
# │       │   ├── adns5050.c
# │       │   ├── Kconfig.adns5050
# │       │   └── CMakeLists.txt
# │       └── CMakeLists.txt
# └── dts/
#     └── bindings/
#         └── sensor/
#             └── pixart,adns5050.yaml

# For module root CMakeLists.txt:
if(CONFIG_ZMK_MODULE_ADNS5050)
  add_subdirectory(drivers)
endif()

# For drivers/CMakeLists.txt:
add_subdirectory_ifdef(CONFIG_INPUT sensor)

# For drivers/sensor/CMakeLists.txt:
add_subdirectory_ifdef(CONFIG_INPUT_ADNS5050 adns5050)

# For drivers/sensor/adns5050/CMakeLists.txt:
zephyr_library_sources_ifdef(CONFIG_INPUT_ADNS5050 adns5050.c)