add_library(
  tracker_keyframe_strategy
  src/fabric.cpp src/frequency_keyframe_strategy.cpp
  src/mean_square_optical_flow_and_rmse_keyframe_strategy.cpp
  src/tracker_keyframe_strategy.cpp)

target_include_directories(
  tracker_keyframe_strategy
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(tracker_keyframe_strategy PUBLIC motion_model time)
target_link_libraries(tracker_keyframe_strategy PRIVATE glog sensors track
                                                        track_frames)
