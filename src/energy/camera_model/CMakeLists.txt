add_library(
  camera_model
  src/camera_model_base.cpp src/pinhole_camera.cpp src/atan_camera.cpp
  src/ios_camera_model.cpp src/simple_radial.cpp)

target_include_directories(
  camera_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(camera_model PUBLIC motion_model time settings ceres)
target_link_libraries(camera_model PRIVATE glog)
