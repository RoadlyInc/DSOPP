add_library(
  marginalization
  src/frame_marginalization_strategy.cpp
  src/maximum_size_frame_marginalization_strategy.cpp
  src/sparse_frame_marginalization_strategy.cpp src/fabric.cpp)

target_include_directories(
  marginalization PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(marginalization PUBLIC motion_model)
target_link_libraries(marginalization PRIVATE glog track track_frames
                                              track_landmarks track_connections)
