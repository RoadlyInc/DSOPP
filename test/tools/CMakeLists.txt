add_library(
  test_tools
  src/tum_gt.cpp src/depth_gt.cpp src/random_track.cpp src/solver_test_data.cpp
  src/compare_track.cpp src/savitzky_golay_filter.cpp)

target_include_directories(
  test_tools PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(
  test_tools
  PUBLIC sophus
         cnpy
         track
         sensors
         tracker
         time
         features
         measures
         camera
         camera_calibration
         settings
         gmock
         common_timestamp_storage)

target_link_libraries(
  test_tools
  PRIVATE glog
          camera_providers
          track_connections
          sensors
          tracker_landmarks_activator
          common_file_tools
          gtest)

get_target_property(TRACKER_INCLUDES tracker INCLUDE_DIRECTORIES)

target_include_directories(test_tools PRIVATE ${TRACKER_INCLUDES})

dsopp_add_test_and_coverage(
  test_track2tum_exporter tests/test_track2tum_exporter.cpp track_export
  test_tools stdc++fs glog)
