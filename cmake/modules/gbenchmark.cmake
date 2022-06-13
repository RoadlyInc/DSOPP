set(URL https://github.com/google/benchmark)
set(VERSION 73d4d5e8d6d449fc8663765a42aa8aeeee844489)
set(gbenchmark_INCLUDE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gbenchmark_external-build/install/include
)

set(gbenchmark_LIBS
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gbenchmark_external-build/install/lib/libbenchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
)

set(gbenchmark_main_LIBS
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gbenchmark_external-build/install/lib/libbenchmark_main${CMAKE_STATIC_LIBRARY_SUFFIX}
)

if(NOT EXISTS ${gbenchmark_LIBS})
  set(gbenchmark_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-restrict -march=native")
  ExternalProject_Add(
    gbenchmark_external
    GIT_REPOSITORY ${URL}
    GIT_TAG ${VERSION}
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DBENCHMARK_ENABLE_TESTING=OFF
      -DBENCHMARK_ENABLE_GTEST_TESTS=OFF
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gbenchmark_external-build/install/
      -DBENCHMARK_ENABLE_INSTALL=ON
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_CXX_FLAGS=${gbenchmark_CXX_FLAGS}
      -DCMAKE_CXX_STANDARD=20
      -DCMAKE_CXX_STANDARD_REQUIRED=ON
    PREFIX 3rd_party
    EXCLUDE_FROM_ALL 1)
endif()

add_library(gbenchmark INTERFACE IMPORTED GLOBAL)
add_dependencies(gbenchmark gbenchmark_external)

target_include_directories(gbenchmark INTERFACE ${gbenchmark_INCLUDE_DIR})
target_link_libraries(gbenchmark INTERFACE ${gbenchmark_LIBS})

add_library(gbenchmark_main INTERFACE IMPORTED GLOBAL)
add_dependencies(gbenchmark_main gbenchmark_external)
file(MAKE_DIRECTORY ${gbenchmark_INCLUDE_DIR})
target_include_directories(gbenchmark_main INTERFACE ${gbenchmark_INCLUDE_DIR})
target_link_libraries(gbenchmark_main INTERFACE ${gbenchmark_main_LIBS} gbenchmark)
