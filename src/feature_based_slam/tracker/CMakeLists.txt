add_library(
  feature_based_slam_tracker
  src/monocular_initializer.cpp
  src/estimate_se3_pnp.cpp
  src/estimate_so3_inlier_count.cpp
  src/estimate_so3xs2.cpp
  src/initialize_poses.cpp
  src/refine_track.cpp
  src/triangulate_points.cpp
  src/monocular_tracker.cpp
  #[[ contact Roadly INC for this functionality
  src/autocalibration_initializer.cpp
  src/autocalibration_selector.cpp
  src/autocalibration_tracker.cpp
  src/estimate_se3_pnp_autocalibration.cpp
  src/estimate_so3xs2_autocalibration.cpp
  ]]
  src/add_new_landmarks.cpp
  src/add_new_projections_to_landmarks.cpp
  src/feature_frame_from_landmarks.cpp
  src/fabric.cpp
  src/remove_outliers.cpp
  src/tracker.cpp
  src/add_new_frame.cpp)

target_include_directories(
  feature_based_slam_tracker
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_include_directories(
  feature_based_slam_tracker
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal/>)

target_link_libraries(
  feature_based_slam_tracker
  PUBLIC common_fabric_tools
         features
         camera_calibration
         glog
         motion_model
         eigen
         time
         settings
         feature_based_slam_initialization_strategy
         feature_based_slam_features)

target_link_libraries(
  feature_based_slam_tracker
  PRIVATE camera_model
          epipolar_geometry
          problems
          ceres
          opengv
          ransac
          n_point_solvers
          sensors
          common_fabric_tools
          common_file_tools)
