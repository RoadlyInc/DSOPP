add_library(
  track_frames
  src/active_keyframe.cpp
  src/frame.cpp
  #[[ contact Roadly INC for this functionality
  src/gnss_frame.cpp
  #]]
  src/keyframe.cpp
  src/slam_internal_tracking_frame.cpp
  src/tracking_frame.cpp)

target_include_directories(
  track_frames PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(track_frames PUBLIC common_patch motion_model features
                                          camera_calibration time proto_storage)
target_link_libraries(track_frames PRIVATE glog track_landmarks
                                           track_connections settings)
