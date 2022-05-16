add_library(output_persistent src/protobuf_exporter.cpp)

target_include_directories(
  output_persistent
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(output_persistent PUBLIC output_interfaces track_frames
                                               motion_model time storage)
target_link_libraries(output_persistent PRIVATE glog track)
