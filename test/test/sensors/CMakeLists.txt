dsopp_add_test_and_coverage(
  test_image_folder_provider camera/providers/test_image_folder_provider.cpp
  camera glog camera_providers settings)

dsopp_add_test_and_coverage(
  test_camera
  camera/test_camera.cpp
  gmock
  gmock_main
  camera
  sophus
  sensors
  camera_calibration
  camera_providers
  features
  settings)

dsopp_add_test(
  test_undistorter
  camera/calibration/undistorter.cpp
  gmock
  gmock_main
  camera
  sophus
  camera_model
  camera_calibration
  glog
  settings)

dsopp_add_test_and_coverage(
  test_get_camera_model
  camera/calibration/test_get_camera_model.cpp
  gmock
  gmock_main
  camera
  camera_providers
  sophus
  camera_model
  camera_calibration
  settings)

dsopp_add_test_and_coverage(
  test_camera_mask
  camera/test_camera_mask.cpp
  gmock
  gmock_main
  sophus
  features
  camera_providers
  camera_calibration
  feature_based_slam_features
  settings)

dsopp_add_test_and_coverage(
  test_image_video_provider camera/providers/test_image_video_provider.cpp
  camera_providers opencv settings)

dsopp_add_test_and_coverage(
  test_npy_folder_provider
  camera/providers/test_npy_folder_provider.cpp
  camera
  glog
  time
  camera_providers
  settings)

dsopp_add_test_and_coverage(
  test_camera_transforms
  camera_transformers/test_camera_transformers.cpp
  glog
  gmock
  gmock_main
  time
  camera
  sensors
  features
  camera_transformers
  camera_providers
  opencv)

dsopp_add_test_and_coverage(
  test_camera_settings
  camera/calibration/test_camera_settings.cpp
  gmock
  gmock_main
  camera
  camera_providers
  camera_model
  camera_calibration
  features
  settings)
