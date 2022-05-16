set(URL https://gitlab.com/libeigen/eigen.git)
set(VERSION 1f4c0311cda3403999b702c996898af5707973a9)

set(eigen_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/eigen_external-build/install/include/eigen3)
get_filename_component(eigen_cmake "${eigen_INCLUDE_DIR}/../../share/eigen3/cmake" ABSOLUTE)
if (NOT EXISTS ${eigen_cmake})
    externalproject_add(eigen_external
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
              -DCMAKE_BUILD_TYPE=Release
              -DCMAKE_INSTALL_PREFIX=./install
            TEST_COMMAND ""
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
    file(MAKE_DIRECTORY ${eigen_INCLUDE_DIR})
endif()

add_library(eigen INTERFACE IMPORTED GLOBAL)
add_dependencies(eigen eigen_external)

target_include_directories(eigen INTERFACE ${eigen_INCLUDE_DIR})
