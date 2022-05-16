add_library(time src/time.cpp)

target_include_directories(
  time PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include/dsopp>)
