add_library(synchronizer src/master_sensor_synchronizer.cpp
                         src/no_synchronization.cpp src/fabric.cpp)

target_include_directories(
  synchronizer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(synchronizer PUBLIC time)
target_link_libraries(synchronizer PRIVATE glog sensors)
