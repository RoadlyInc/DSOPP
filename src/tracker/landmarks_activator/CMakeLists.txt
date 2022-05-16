add_library(tracker_landmarks_activator src/landmarks_activator.cpp)

target_include_directories(
  tracker_landmarks_activator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(
  tracker_landmarks_activator PUBLIC motion_model measures features
                                     camera_calibration settings)
target_link_libraries(
  tracker_landmarks_activator PRIVATE camera_projector track track_frames
                                      track_landmarks problems)
