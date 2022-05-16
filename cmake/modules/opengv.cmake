set(URL  https://github.com/laurentkneip/opengv.git)
set(VERSION 306a54e6c6b94e2048f820cdf77ef5281d4b48ad)

set(opengv_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/opengv_external/include)
set(opengv_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/opengv_external-build/lib/libopengv${CMAKE_STATIC_LIBRARY_SUFFIX})

get_target_property(eigen_INCLUDE_DIRS eigen INTERFACE_INCLUDE_DIRECTORIES)

if (NOT EXISTS ${opengv_LIBS})
    externalproject_add(opengv_external
            DEPENDS eigen
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
                -DEIGEN_INCLUDE_DIR:PATH=${eigen_INCLUDE_DIRS}
            INSTALL_COMMAND ""
            TEST_COMMAND ""
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
endif()
add_library(opengv INTERFACE IMPORTED GLOBAL)
add_dependencies(opengv opengv_external)
file(MAKE_DIRECTORY ${opengv_INCLUDE_DIR})
target_include_directories(opengv INTERFACE ${opengv_INCLUDE_DIR} ${eigen_INCLUDE_DIR})
target_link_libraries(opengv INTERFACE ${opengv_LIBS} )
