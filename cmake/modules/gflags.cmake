set(URL https://github.com/gflags/gflags.git)
set(VERSION 827c769e5fc98e0f2a34c47cef953cc6328abced)
set(gflags_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gflags_external-build/install/include)
set(gflags_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gflags_external-build/install/lib/libgflags${CMAKE_STATIC_LIBRARY_SUFFIX})
if (NOT EXISTS ${gflags_LIBS})
    externalproject_add(gflags_external
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
                -DGFLAGS_NAMESPACE=google
                -DCMAKE_CXX_FLAGS=-fPIC
                -DBUILD_SHARED_LIBS=OFF
                -DBUILD_STATIC_LIBS=ON
                -DBUILD_PACKAGING=OFF
                -DBUILD_TESTING=OFF
                -DINSTALL_HEADERS=ON
                -DREGISTER_INSTALL_PREFIX=OFF
                -DCMAKE_INSTALL_PREFIX=./install
            TEST_COMMAND ""
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
endif()

add_library(gflags INTERFACE IMPORTED GLOBAL)
add_dependencies(gflags gflags_external)
file(MAKE_DIRECTORY ${gflags_INCLUDE_DIR})
target_include_directories(gflags INTERFACE ${gflags_INCLUDE_DIR})
set_property(GLOBAL PROPERTY gflags_path ${gflags_LIBS})
target_link_libraries(gflags INTERFACE ${gflags_LIBS})
