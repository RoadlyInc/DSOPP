add_executable(feature_frame_benchmark feature_frame_benchmark.cpp)

target_link_libraries(
  feature_frame_benchmark
  features
  sensors
  eigen
  sensors
  gbenchmark_main
  settings)
