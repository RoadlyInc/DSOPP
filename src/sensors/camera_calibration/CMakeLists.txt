add_library(
  camera_calibration src/fabric.cpp src/undistorter.cpp src/camera_mask.cpp
                     src/camera_settings.cpp src/camera_calibration.cpp)

target_include_directories(
  camera_calibration
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(
  camera_calibration PUBLIC camera_model common_semantics time opencv
                            proto_storage camera_model)
target_link_libraries(camera_calibration PRIVATE camera_transformers
                                                 common_file_tools glog)
