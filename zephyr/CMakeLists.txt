# モジュールのルートCMakeLists.txt
if(CONFIG_ADNS5050)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../drivers ${CMAKE_CURRENT_BINARY_DIR}/drivers)
endif()