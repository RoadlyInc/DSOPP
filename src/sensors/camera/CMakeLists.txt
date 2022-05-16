add_library(camera src/camera.cpp)

target_include_directories(
  camera PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(camera PUBLIC sensor camera_calibration settings
                                    camera_transformers common_semantics)
target_link_libraries(camera PRIVATE features camera_providers glog)
