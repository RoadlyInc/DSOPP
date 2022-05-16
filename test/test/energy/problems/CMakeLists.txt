get_target_property(TRACKER_INCLUDES tracker INCLUDE_DIRECTORIES)
get_target_property(PROBLEMS_INCLUDES problems INCLUDE_DIRECTORIES)

dsopp_add_test(
  test_affine_brightness
  test_affine_brightness.cpp
  problems
  sensors
  features
  test_tools
  ceres
  eigen
  time
  gmock
  settings)

target_include_directories(test_affine_brightness PRIVATE ${TRACKER_INCLUDES})

dsopp_add_test(
  test_ceres_pose_alignment
  test_ceres_pose_alignment.cpp
  problems
  sensors
  test_tools
  ceres
  eigen
  time
  tracker
  marginalization
  feature_based_slam_tracker
  settings)

target_include_directories(test_ceres_pose_alignment
                           PRIVATE ${TRACKER_INCLUDES})

dsopp_add_test(
  test_photometric_bundle_adjustment
  test_photometric_bundle_adjustment.cpp
  problems
  sensors
  test_tools
  ceres
  eigen
  time
  tracker
  settings)

dsopp_add_test_and_coverage(test_local_parameterization_s2
                            test_local_parameterization_s2.cpp problems)

target_include_directories(test_local_parameterization_s2
                           PRIVATE ${PROBLEMS_INCLUDES})

dsopp_add_test(
  test_ceres_pose_alignment_noised
  test_ceres_pose_alignment_rotated.cpp
  problems
  sensors
  features
  test_tools
  ceres
  eigen
  gmock
  tracker
  settings)

target_include_directories(test_ceres_pose_alignment_noised
                           PRIVATE ${TRACKER_INCLUDES})

target_include_directories(test_ceres_pose_alignment_noised
                           PRIVATE ${TRACKER_INCLUDES})

dsopp_add_test(
  test_analytical_diff
  test_analytical_diff.cpp
  problems
  sensors
  test_tools
  ceres
  eigen
  gmock
  test_tools
  settings)
target_include_directories(test_analytical_diff PRIVATE ${PROBLEMS_INCLUDES})

dsopp_add_test(
  test_linear_system
  test_linear_system.cpp
  problems
  problems
  sensors
  test_tools
  ceres
  eigen
  time
  tracker
  settings)
target_include_directories(test_linear_system PRIVATE ${PROBLEMS_INCLUDES})

dsopp_add_test(
  test_incremental_solver
  test_incremental_solver.cpp
  problems
  sensors
  test_tools
  ceres
  eigen
  time
  tracker
  settings)

dsopp_add_test(
  test_ceres_camera_optimization
  test_ceres_camera_optimization.cpp
  problems
  sensors
  test_tools
  ceres
  eigen
  time
  tracker)

target_include_directories(test_ceres_camera_optimization
                           PRIVATE ${PROBLEMS_INCLUDES})

dsopp_add_test_and_coverage(
  test_geometric_bundle_adjustment test_geometric_bundle_adjustment.cpp
  problems camera_model motion_model camera_calibration)
