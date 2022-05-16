add_library(dsopp src/dsopp.cpp src/config_loader.cpp)

target_include_directories(
  dsopp PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
               $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(dsopp PUBLIC motion_model output_interfaces)

target_link_libraries(
  dsopp
  PRIVATE agent
          synchronizer
          problems
          sanity_checker
          marginalization
          feature_based_slam_tracker
          features
          yaml-cpp
          sensors
          sensors_builder
          camera_calibration
          tracker
          tracker_depth_estimators
          measures
          glog
          stdc++fs
          track)
