add_library(storage src/track_storage.cpp)

target_include_directories(
  storage PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(storage PUBLIC output_interfaces track_frames
                                     motion_model time sanity_check_status)
target_link_libraries(storage PRIVATE proto_storage)

add_subdirectory(proto)
