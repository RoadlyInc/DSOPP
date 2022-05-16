add_library(track_connections src/connections_container.cpp
                              src/frame_connection.cpp)

target_include_directories(
  track_connections
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(track_connections PUBLIC motion_model proto_storage
                                               settings)
