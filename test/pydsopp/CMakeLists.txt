add_subdirectory(features)
add_subdirectory(python_reader)

set(EXPORT_TEST_DATA "${CMAKE_CURRENT_SOURCE_DIR}/../test_data/pydsopp/export")
set(SMALL_TRACK_BIN "${EXPORT_TEST_DATA}/small_track.bin")
set(SMALL_TRACK_JSON "${EXPORT_TEST_DATA}/small_track.json")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/test_json_exporter.sh"
               "${CMAKE_CURRENT_BINARY_DIR}/test_json_exporter.sh")
add_test(test_pydsopp_json_exporter bash
         "${CMAKE_CURRENT_BINARY_DIR}/test_json_exporter.sh")
