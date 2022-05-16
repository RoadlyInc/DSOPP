add_executable(save_random_track save_random_track.cpp)
target_link_libraries(save_random_track PUBLIC track test_tools storage
                                               motion_model sensors)

add_executable(compare_two_tracks compare_two_tracks.cpp)
target_link_libraries(compare_two_tracks PUBLIC test_tools storage sensors)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/read_and_save.py"
               "${CMAKE_CURRENT_BINARY_DIR}/read_and_save.py" COPYONLY)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/test_python_reader.sh"
               "${CMAKE_CURRENT_BINARY_DIR}/test_python_reader.sh" COPYONLY)

add_test(test_python_reader bash
         "${CMAKE_CURRENT_BINARY_DIR}/test_python_reader.sh")
