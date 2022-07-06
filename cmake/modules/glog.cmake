set(URL https://github.com/google/glog.git)
set(VERSION 0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6)
set(glog_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/glog_external-build/install/include)
set(glog_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/glog_external-build/install/lib/libglog${CMAKE_STATIC_LIBRARY_SUFFIX})
if (NOT EXISTS ${glog_LIBS})
    externalproject_add(glog_external
            DEPENDS gflags
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
                -DCMAKE_CXX_STANDARD=20
                -DCMAKE_CXX_STANDARD_REQUIRED=ON
                -DCMAKE_CXX_FLAGS=-march=native
                -DCMAKE_INSTALL_PREFIX=./install
                -DBUILD_TESTING=OFF
                -DWITH_GFLAGS=OFF
                -DWITH_UNWIND=OFF
            TEST_COMMAND ""
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
endif()

add_library(glog INTERFACE IMPORTED GLOBAL)
add_dependencies(glog glog_external)
file(MAKE_DIRECTORY ${glog_INCLUDE_DIR})
target_include_directories(glog INTERFACE ${glog_INCLUDE_DIR})
set_property(GLOBAL PROPERTY glog_path ${glog_LIBS})
target_link_libraries(glog INTERFACE ${glog_LIBS} gflags ${GFLAGS_LIBRARIES})
