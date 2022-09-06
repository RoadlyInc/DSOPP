add_library(common_file_tools src/parsing.cpp src/read_tum_poses.cpp
                              src/camera_frame_times.cpp)

target_include_directories(
  common_file_tools
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(common_file_tools PUBLIC motion_model settings stdc++fs
                                               time glog)
