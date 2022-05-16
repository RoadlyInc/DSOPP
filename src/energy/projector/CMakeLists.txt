add_library(camera_projector INTERFACE)

target_include_directories(
  camera_projector
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(camera_projector INTERFACE camera_model motion_model)
