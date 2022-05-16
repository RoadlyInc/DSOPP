set(URL https://github.com/jbeder/yaml-cpp.git)
set(VERSION 08aa252611843b93c4f98959fe89c81b872224ae)

set(yaml-cpp_INCLUDE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/yaml-cpp_external/include)
set(yaml-cpp_LIBS
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/yaml-cpp_external-build/libyaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}
)

file(MAKE_DIRECTORY ${yaml-cpp_INCLUDE_DIR})


ExternalProject_Add(
  yaml-cpp_external
  GIT_REPOSITORY ${URL}
  GIT_TAG ${VERSION}
  INSTALL_COMMAND ""
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  TEST_COMMAND ""
  PREFIX 3rd_party
  EXCLUDE_FROM_ALL 1)

add_library(yaml-cpp INTERFACE IMPORTED GLOBAL)
add_dependencies(yaml-cpp yaml-cpp_external)
target_include_directories(yaml-cpp INTERFACE ${yaml-cpp_INCLUDE_DIR})
target_link_libraries(yaml-cpp INTERFACE ${yaml-cpp_LIBS})
