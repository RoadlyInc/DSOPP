add_library(camera_transformers src/camera_resizer.cpp src/image_cropper.cpp
                                src/fabric.cpp src/camera_transformer.cpp)

target_include_directories(
  camera_transformers
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(camera_transformers PUBLIC sensor camera_calibration
                                                 settings opencv)
target_link_libraries(camera_transformers PRIVATE features common_file_tools
                                                  glog)
