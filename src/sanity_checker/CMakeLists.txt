add_library(sanity_check_status INTERFACE)

target_include_directories(
  sanity_check_status
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/dsopp>)

add_library(
  sanity_checker
  src/fabric.cpp
  #[[ contact Roadly INC for this functionality
src/ackermann_checker.cpp)
]]
)

target_include_directories(
  sanity_checker PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(sanity_checker PUBLIC motion_model)
target_link_libraries(
  sanity_checker
  PRIVATE track
          common_fabric_tools
          common_file_tools
          settings
          time
          sophus
          glog)
