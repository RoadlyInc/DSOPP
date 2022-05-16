set(GENERATED_TEST_DATA_DIR ${PROJECT_BINARY_DIR}/test_tmp)

set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test_data")
get_target_property(DSOPP_SOURCE_PATH dsopp_main BINARY_DIR)
set(PYTHON_EVALUATOR_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/evaluation/evaluate_ate.py")

add_compile_definitions(GENERATED_TEST_DATA_DIR="${GENERATED_TEST_DATA_DIR}")
add_compile_definitions(TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")

file(MAKE_DIRECTORY ${GENERATED_TEST_DATA_DIR})

include_directories(${CMAKE_SOURCE_DIR}/src)

include_directories(${CMAKE_SOURCE_DIR}/test/mock)
include_directories(${CMAKE_SOURCE_DIR}/test/tools)

add_subdirectory(tools)
add_subdirectory(performance)
add_subdirectory(test)
add_subdirectory(pydsopp)

set(DATASET_MONO
    "${CMAKE_CURRENT_SOURCE_DIR}/test_data/track30seconds/mono.yaml")
set(DATASET_GT_TUM
    "${CMAKE_CURRENT_SOURCE_DIR}/test_data/track30seconds/gt.tum")
set(DATASET_GT_ENU
    "${CMAKE_CURRENT_SOURCE_DIR}/test_data/track30seconds/gt.enu")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/local_track_performance_test.sh"
               "${CMAKE_CURRENT_BINARY_DIR}/local_track_performance_test.sh")
