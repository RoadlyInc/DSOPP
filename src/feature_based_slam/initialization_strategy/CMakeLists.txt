add_library(
  feature_based_slam_initialization_strategy
  src/frequency_initializer_keyframe_strategy.cpp
  src/wait_for_movement_keyframe_strategy.cpp)

target_include_directories(
  feature_based_slam_initialization_strategy
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(
  feature_based_slam_initialization_strategy
  PUBLIC sophus feature_based_slam_track feature_based_slam_features)
target_link_libraries(feature_based_slam_initialization_strategy
                      PRIVATE time settings features)
