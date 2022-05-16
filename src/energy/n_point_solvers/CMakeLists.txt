add_library(n_point_solvers src/pure_rotation_estimator.cpp)

target_include_directories(
  n_point_solvers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/dsopp>)

target_include_directories(n_point_solvers PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(n_point_solvers PUBLIC eigen epipolar_geometry
                                             motion_model)

target_link_libraries(n_point_solvers PRIVATE glog)
