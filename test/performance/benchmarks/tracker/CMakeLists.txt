add_executable(depth_estimation_benchmark depth_estimation_benchmark.cpp)

get_target_property(TRACKER_INCLUDES tracker INCLUDE_DIRECTORIES)

target_include_directories(depth_estimation_benchmark
                           PRIVATE ${TRACKER_INCLUDES})

target_link_libraries(
  depth_estimation_benchmark
  gbenchmark_main
  tracker
  features
  epipolar_geometry
  sensors
  camera
  test_tools
  settings
  sophus
  time)
