add_executable(photometric_bundle_adjustment_benchmark
               photometric_bundle_adjustment_benchmark.cpp)

get_target_property(PROBLEMS_INCLUDES problems INCLUDE_DIRECTORIES)

target_include_directories(photometric_bundle_adjustment_benchmark
                           PRIVATE ${PROBLEMS_INCLUDES})
target_link_libraries(
  photometric_bundle_adjustment_benchmark
  problems
  sensors
  test_tools
  ceres
  eigen
  time
  tracker
  gbenchmark_main
  settings)

add_executable(pose_alignment_benchmark pose_alignment_benchmark.cpp)
target_link_libraries(
  pose_alignment_benchmark
  problems
  sensors
  test_tools
  ceres
  eigen
  time
  tracker
  gbenchmark_main
  settings)
