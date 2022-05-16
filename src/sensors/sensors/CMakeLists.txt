add_library(sensors src/sensors.cpp)

target_include_directories(
  sensors PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(sensors PUBLIC camera)
target_link_libraries(sensors PRIVATE glog features)
