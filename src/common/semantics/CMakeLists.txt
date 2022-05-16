add_library(common_semantics src/semantic_legend.cpp src/semantic_filter.cpp)

target_include_directories(
  common_semantics PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(common_semantics PUBLIC eigen)
target_link_libraries(common_semantics PRIVATE glog common_file_tools
                                               proto_storage)
