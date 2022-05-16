add_library(ransac INTERFACE)

target_include_directories(
  ransac INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(ransac INTERFACE eigen sophus n_point_solvers)
