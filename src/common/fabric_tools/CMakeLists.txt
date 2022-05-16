add_library(common_fabric_tools src/parameters.cpp)

target_include_directories(
  common_fabric_tools
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(common_fabric_tools PUBLIC glog)
