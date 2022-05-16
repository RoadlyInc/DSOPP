if(VISUALIZATION)
  add_library(
    visualizer
    src/visualizer_track_output_interface.cpp
    src/visualizer_image_output_interface.cpp
    src/visualizer_text_output_interface.cpp
    src/local_track.cpp
    src/local_frame.cpp
    src/local_odometry_track.cpp
    src/local_odometry_frame.cpp
    src/local_tracking_frame.cpp
    src/visualizer.cpp)

  target_include_directories(
    visualizer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/dsopp>)

  target_include_directories(
    visualizer PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal/>)

  target_link_libraries(visualizer PUBLIC output_interfaces track_frames
                                          pangolin motion_model time)
  target_link_libraries(visualizer PRIVATE glog track)
endif()
