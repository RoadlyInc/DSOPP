add_library(common_patch INTERFACE)

target_include_directories(
  common_patch INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(common_patch INTERFACE eigen)
