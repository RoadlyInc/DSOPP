add_library(
  feature_based_slam_features
  src/distinct_feature.cpp src/distinct_features_frame.cpp
  src/distinct_features_extractor_orb.cpp src/optical_flow.cpp
  src/correspondences_finder.cpp src/orb.cpp)

target_include_directories(
  feature_based_slam_features
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(feature_based_slam_features PUBLIC eigen settings
                                                         camera_calibration)
