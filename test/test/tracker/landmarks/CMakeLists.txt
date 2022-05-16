get_target_property(TRACKER_INCLUDES tracker INCLUDE_DIRECTORIES)

dsopp_add_test(
  test_depth_estimation
  test_depth_estimation.cpp
  gmock
  gmock_main
  tracker
  features
  sensors
  test_tools
  $<$<BOOL:${VISUALIZATION}>:visualizer>
  epipolar_geometry
  glog
  common_image_tools
  sophus
  settings)

target_include_directories(test_depth_estimation PRIVATE ${TRACKER_INCLUDES})

dsopp_add_test(
  test_landmarks_activator
  test_landmarks_activator.cpp
  gmock
  gmock_main
  tracker
  marginalization
  features
  sensors
  test_tools
  $<$<BOOL:${VISUALIZATION}>:visualizer>
  glog
  sophus
  settings)

target_include_directories(test_landmarks_activator PRIVATE ${TRACKER_INCLUDES})
