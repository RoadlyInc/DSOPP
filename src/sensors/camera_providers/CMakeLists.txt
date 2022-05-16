add_library(
  camera_providers
  src/camera_data_frame.cpp src/fabric.cpp src/image_folder_provider.cpp
  src/image_video_provider.cpp src/npy_folder_provider.cpp)

target_include_directories(
  camera_providers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(camera_providers PUBLIC camera_calibration time sensor
                                              common_file_tools)
target_link_libraries(camera_providers PRIVATE glog stdc++fs cnpy)
