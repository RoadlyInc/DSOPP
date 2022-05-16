add_library(tracker_depth_estimators src/depth_estimation.cpp)

target_include_directories(
  tracker_depth_estimators
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(
  tracker_depth_estimators PUBLIC motion_model measures camera_calibration
                                  track_landmarks features settings)
target_link_libraries(
  tracker_depth_estimators PRIVATE glog camera_projector common_pattern
                                   problems epipolar_geometry tbb)
