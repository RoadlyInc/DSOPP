add_subdirectory(constexpr_tools)
add_subdirectory(time)
add_subdirectory(pattern)
add_subdirectory(patch)
add_subdirectory(image_tools)
add_subdirectory(file_tools)
add_subdirectory(fabric_tools)
add_subdirectory(semantics)
add_subdirectory(timestamp_storage)

add_library(settings INTERFACE)

target_include_directories(
  settings INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include/dsopp>)
