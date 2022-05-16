add_library(
  track
  src/active_odometry_track.cpp
  src/active_track.cpp
  #[[ contact Roadly INC for this functionality
  src/ecef_poses.cpp
  src/gnss_track.cpp
  ]]
  src/odometry_track.cpp
  src/odometry_track_base.cpp
  src/track.cpp
  src/track_base.cpp)

target_include_directories(
  track PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
               $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(track PUBLIC motion_model track_frames proto_storage
                                   storage sanity_check_status agent)
target_link_libraries(track PRIVATE glog track_connections track_landmarks
                                    settings tbb)
