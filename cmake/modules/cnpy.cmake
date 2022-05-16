set(URL  https://github.com/rogersce/cnpy.git)
set(VERSION 4e8810b1a8637695171ed346ce68f6984e585ef4)
set(cnpy_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/cnpy_external)
set(cnpy_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/cnpy_external-build/libcnpy${CMAKE_STATIC_LIBRARY_SUFFIX})

find_package(ZLIB REQUIRED)
if (NOT EXISTS ${cnpy_LIBS})
    externalproject_add(cnpy_external
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
                -DCMAKE_INSTALL_PREFIX=./install
            INSTALL_COMMAND ""
            TEST_COMMAND ""
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
endif()
add_library(cnpy INTERFACE IMPORTED GLOBAL)
add_dependencies(cnpy cnpy_external)
file(MAKE_DIRECTORY ${cnpy_INCLUDE_DIR})
target_include_directories(cnpy INTERFACE ${cnpy_INCLUDE_DIR})
target_link_libraries(cnpy INTERFACE ${ZLIB_LIBRARIES})
target_link_libraries(cnpy INTERFACE ${cnpy_LIBS} ${ZLIB_LIBRARIES})
