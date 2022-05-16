add_library(application_tools src/read_tracks.cpp)

target_include_directories(
  application_tools
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(application_tools PUBLIC track motion_model settings
                                               stdc++fs storage)
