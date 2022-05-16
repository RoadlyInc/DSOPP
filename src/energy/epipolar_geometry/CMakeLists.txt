add_library(
  epipolar_geometry
  src/epipolar_line.cpp src/epipolar_line_builder.cpp
  src/se3_epipolar_line_triangulator.cpp src/essential_matrix.cpp)

target_include_directories(
  epipolar_geometry
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(epipolar_geometry PUBLIC camera_projector settings)
target_link_libraries(epipolar_geometry PRIVATE glog)
