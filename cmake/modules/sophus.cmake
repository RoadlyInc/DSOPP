set(URL https://github.com/strasdat/Sophus.git)
set(VERSION 593db47500ea1a2de5f0e6579c86147991509c59)

set(sophus_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/sophus_external)

get_target_property(eigen_INCLUDE_DIRS eigen INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(eigen_cmake "${eigen_INCLUDE_DIRS}/../../share/eigen3/cmake" ABSOLUTE)


if (NOT EXISTS ${sophus_INCLUDE_DIR}/sophus)
    externalproject_add(sophus_external
            DEPENDS eigen
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
                -DBUILD_TESTS=OFF
                -DEigen3_DIR=${eigen_cmake}
            CONFIGURE_COMMAND ""
            BUILD_COMMAND ""
            INSTALL_COMMAND ""
            TEST_COMMAND ""
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
    file(MAKE_DIRECTORY ${sophus_INCLUDE_DIR})
endif()

add_library(sophus INTERFACE IMPORTED GLOBAL)
add_dependencies(sophus sophus_external)

target_include_directories(sophus INTERFACE ${sophus_INCLUDE_DIR} ${eigen_INCLUDE_DIR})
