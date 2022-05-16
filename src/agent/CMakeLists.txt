add_library(agent src/agent.cpp src/agent_settings.cpp)

target_include_directories(
  agent PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
               $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(agent PUBLIC sensors camera_calibration proto_storage
                                   settings)
target_link_libraries(agent PRIVATE camera glog)
