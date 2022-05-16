add_library(tracker src/fabric.cpp src/monocular_tracker.cpp
                    src/create_depth_maps.cpp)

target_include_directories(
  tracker PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:include/dsopp>)

target_include_directories(
  tracker PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal/>)

target_link_libraries(
  tracker
  PUBLIC motion_model
         problems
         tracker_depth_estimators
         tracker_keyframe_strategy
         tracker_landmarks_activator
         features
         measures
         sensors
         camera_providers
         tracker_depth_estimators
         camera_calibration
         settings)
target_link_libraries(
  tracker
  PRIVATE feature_based_slam_tracker
          camera_projector
          marginalization
          track
          track_frames
          track_landmarks
          camera
          glog
          output_interfaces
          common_image_tools
          common_fabric_tools
          common_file_tools)
