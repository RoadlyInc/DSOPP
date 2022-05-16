dsopp_add_test(
  test_reference_frame_depth_map
  test_reference_frame_depth_map.cpp
  gmock
  gmock_main
  tracker
  features
  sensors
  test_tools
  marginalization
  feature_based_slam_tracker
  sophus
  settings)

get_target_property(TRACKER_INCLUDES tracker INCLUDE_DIRECTORIES)
target_include_directories(test_reference_frame_depth_map
                           PRIVATE ${TRACKER_INCLUDES})
