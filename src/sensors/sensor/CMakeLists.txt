add_library(sensor src/synchronized_frame.cpp)

target_include_directories(
  sensor PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(sensor PUBLIC motion_model time settings)
target_link_libraries(sensor PRIVATE glog features)
