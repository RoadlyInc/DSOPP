add_library(
  track_export src/track2tum_exporter.cpp src/transformations_output.cpp
               src/timestamp_output.cpp)

target_include_directories(
  track_export PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(track_export PUBLIC motion_model)
target_link_libraries(track_export PRIVATE glog track_frames track)
