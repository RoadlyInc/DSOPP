add_library(common_timestamp_storage src/timestamp_storage.cpp)

target_include_directories(
  common_timestamp_storage
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(common_timestamp_storage PUBLIC motion_model time
                                                      settings)
