add_library(common_pattern INTERFACE)

target_include_directories(
  common_pattern
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(common_pattern INTERFACE eigen settings)
