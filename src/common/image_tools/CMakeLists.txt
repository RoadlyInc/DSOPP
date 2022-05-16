add_library(common_image_tools src/conversion.cpp)

target_include_directories(
  common_image_tools
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(common_image_tools PUBLIC features)
target_link_libraries(common_image_tools PRIVATE settings eigen)
