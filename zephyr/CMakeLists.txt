# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT

if(CONFIG_ADNS5050)
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../drivers ${CMAKE_CURRENT_BINARY_DIR}/drivers)
endif()