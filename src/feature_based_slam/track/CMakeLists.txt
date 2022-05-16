add_library(feature_based_slam_track src/frame.cpp src/landmark.cpp
                                     src/landmarks.cpp)

target_include_directories(
  feature_based_slam_track
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(feature_based_slam_track
                      PUBLIC sophus feature_based_slam_features)
target_link_libraries(feature_based_slam_track PRIVATE time settings features)
