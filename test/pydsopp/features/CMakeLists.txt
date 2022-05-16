get_target_property(PYDSOPP_DIR pydsopp BINARY_DIR)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/test_extract_tracking_features.py"
               "${CMAKE_CURRENT_BINARY_DIR}/test_extract_tracking_features.py")

add_test(test_extract_tracking_features python3
         "${CMAKE_CURRENT_BINARY_DIR}/test_extract_tracking_features.py")
