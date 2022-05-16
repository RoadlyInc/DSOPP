dsopp_add_test(test_config_loader test_config_loader.cpp stdc++fs yaml-cpp
               dsopp settings)

add_compile_definitions(
  MONO_CONFIG_FILE="${CMAKE_SOURCE_DIR}/test/test_data/track30seconds/mono.yaml"
)
