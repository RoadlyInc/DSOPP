add_executable(reprojector_benchmark reprojector_benchmark.cpp)

target_link_libraries(
  reprojector_benchmark
  camera_model
  sensors
  eigen
  gbenchmark_main
  camera_projector
  camera_calibration
  settings)
