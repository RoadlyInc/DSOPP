dsopp_add_test(
  test_pinhole_camera_model
  test_pinhole_camera_model.cpp
  camera_model
  sophus
  epipolar_geometry
  ceres
  opencv
  settings)

dsopp_add_test_and_coverage(test_atan_camera_model test_atan_camera_model.cpp
                            camera_model sophus ceres settings)

dsopp_add_test_and_coverage(test_ios_camera_model test_ios_camera_model.cpp
                            camera_model sophus ceres settings)

dsopp_add_test_and_coverage(test_simple_radial test_simple_radial.cpp
                            camera_model sophus ceres settings)

dsopp_add_test_and_coverage(
  test_division_camera_model test_division_camera_model.cpp camera_model sophus
  ceres settings)

dsopp_add_test_and_coverage(test_tum_fov_model test_tum_fov_model.cpp
                            camera_model sophus ceres settings)
