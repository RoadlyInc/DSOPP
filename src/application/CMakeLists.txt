add_subdirectory(tools)

# Get the latest abbreviated commit hash of the working branch
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE)

add_compile_definitions(GIT_COMMIT_HASH="${GIT_COMMIT_HASH}")
add_compile_definitions(TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/test_data/")

add_compile_definitions(
  MONO_CONFIG_FILE="${CMAKE_SOURCE_DIR}/test/test_data/track30seconds/mono.yaml"
)

add_executable(dsopp_main dsopp_main.cpp)
target_link_libraries(
  dsopp_main PRIVATE dsopp $<$<BOOL:${VISUALIZATION}>:visualizer>
                     output_persistent glog tbb)

if(VISUALIZATION)
  add_executable(viewer_main viewer_main.cpp)
  target_link_libraries(viewer_main PRIVATE track visualizer application_tools
                                            sensors)
endif()
#[[contact Roadly INC for this functionality

add_executable(dsopp_main_rolling_shutter dsopp_main.cpp)
target_link_libraries(
  dsopp_main_rolling_shutter
  PRIVATE dsopp $<$<BOOL:${VISUALIZATION}>:visualizer> output_persistent glog
          tbb)
target_compile_definitions(dsopp_main_rolling_shutter
                           PRIVATE ROLLING_SHUTTER_BUILD)

add_executable(gnss_align gnss_align.cpp)
target_link_libraries(
  gnss_align
  PRIVATE track
          $<$<BOOL:${VISUALIZATION}>:visualizer>
          application_tools
          sensors
          output_persistent)


add_executable(mapper_main mapper_main.cpp)
target_link_libraries(
  mapper_main
  PRIVATE track
          $<$<BOOL:${VISUALIZATION}>:visualizer>
          application_tools
          sensors
          relocalization_candidates
          camera_correspondence
          relocalization_pose_estimation
          global_localization
          gnss_position_constraint
          relocalization_constraint
          output_persistent)

add_executable(sanity_checks sanity_checks.cpp)
target_link_libraries(sanity_checks PRIVATE common_file_tools settings track
                                            sanity_checker storage)


]]

add_executable(track2trajectory track2trajectory.cpp)
target_link_libraries(track2trajectory PRIVATE track_export test_tools settings
                                               storage)
