add_library(
  motion_model
  src/motion.cpp src/se3_motion.cpp
  #[[ contact Roadly INC for this functionality
                         src/uniform_rolling_shutter.cpp
]]
)

target_include_directories(
  motion_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(motion_model PUBLIC eigen sophus)
target_link_libraries(motion_model PRIVATE glog ceres)
