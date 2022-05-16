add_library(
  problems
  src/ceres_photometric_bundle_adjustment.cpp
  src/geometric_bundle_adjustment/ceres_geometric_bundle_adjustment_solver.cpp
  src/geometric_bundle_adjustment/local_frame.cpp
  src/so3xs2_refinement.cpp
  src/bundle_adjustment_photometric_evaluation_callback.cpp
  src/photometric_bundle_adjustment.cpp
  src/eigen_pose_alignment.cpp
  src/eigen_photometric_bundle_adjustment.cpp
  src/normal_linear_system.cpp
  src/void_photometric_bundle_adjustment.cpp
  src/precalculated_pose_alignment.cpp)

target_include_directories(
  problems PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include/dsopp>)

target_include_directories(
  problems PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal/>)

target_link_libraries(
  problems
  PUBLIC motion_model
         ceres
         common_pattern
         common_patch
         features
         measures
         camera_calibration
         camera_projector
         track_connections
         track_frames
         $<$<BOOL:${COMPILE_HIDDEN_CODE}>:common_image_tools>
         camera
         common_timestamp_storage)

target_link_libraries(problems PRIVATE glog ceres track_landmarks tbb)
