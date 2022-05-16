add_library(features_wrapper src/extract_tracking_features.cpp)

target_include_directories(
  features_wrapper PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include/dsopp>)

target_link_libraries(features_wrapper PUBLIC settings pybind11)
target_link_libraries(features_wrapper PRIVATE features)
