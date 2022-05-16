# add_library( test_depth_estimation_accuracy OBJECT
# test_depth_estimation_accuracy.cpp)
add_executable(test_depth_estimation_accuracy
               test_depth_estimation_accuracy.cpp)

get_target_property(TRACKER_INCLUDES tracker INCLUDE_DIRECTORIES)

target_include_directories(test_depth_estimation_accuracy
                           PRIVATE ${TRACKER_INCLUDES})

target_link_libraries(
  test_depth_estimation_accuracy
  tracker
  features
  epipolar_geometry
  sensors
  camera
  test_tools
  settings
  sophus
  time)
