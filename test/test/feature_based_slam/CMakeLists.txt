dsopp_add_test(test_fbs_feature_matcher test_fbs_feature_matcher.cpp
               feature_based_slam_features test_tools common_image_tools)

dsopp_add_test(
  test_fbs_tracker
  test_fbs_tracker.cpp
  gmock_main
  tracker
  features
  sensors
  test_tools
  ransac
  sophus
  ceres
  feature_based_slam_tracker
  settings)
