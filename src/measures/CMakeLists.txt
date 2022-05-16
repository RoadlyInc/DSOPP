add_library(measures INTERFACE)

target_include_directories(
  measures INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(measures INTERFACE common_pattern eigen)
