add_library(output_interfaces INTERFACE)

target_include_directories(
  output_interfaces
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(output_interfaces INTERFACE motion_model agent)
