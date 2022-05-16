add_library(
  sensors_builder
  src/camera_fabric.cpp
  #[[contact Roadly INC for this functionality";
 src/gnss_fabric.cpp
 src/imu_fabric.cpp
 ]]
)

target_include_directories(
  sensors_builder PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(sensors_builder PUBLIC camera camera_transformers)
target_link_libraries(
  sensors_builder PRIVATE glog camera_providers features common_fabric_tools
                          common_file_tools)
