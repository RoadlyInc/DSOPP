add_library(
  track_landmarks
  src/active_tracking_landmark.cpp src/immature_tracking_landmark.cpp
  src/tracking_landmark.cpp src/tracking_landmark_base.cpp
  src/tracking_landmark_with_patch.cpp)

target_include_directories(
  track_landmarks PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(track_landmarks PUBLIC common_patch common_pattern
                                             proto_storage common_semantics)
target_link_libraries(track_landmarks PRIVATE glog settings)
