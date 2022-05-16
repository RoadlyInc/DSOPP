# Wrapper to simplify test
function(dsopp_add_test test_name test_source)
  add_executable(${test_name} ${test_source})
  target_link_libraries(${test_name} gtest gtest_main)
  target_link_libraries(${test_name} ${ARGN})
  add_test(${test_name} ${test_name})
endfunction()

# Wrapper to simplify test and coverage
function(dsopp_add_test_and_coverage test_name test_source)
  dsopp_add_test(${test_name} ${test_source} ${ARGN})
  set_tests_properties(${test_name} PROPERTIES LABELS "Coverage")
endfunction()
