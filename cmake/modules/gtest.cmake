set(URL https://github.com/google/googletest)
set(VERSION aee0f9d9b5b87796ee8a0ab26b7587ec30e8858e)
set(gtest_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external/googletest/include)
set(gtest_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external-build/lib/libgtest${CMAKE_STATIC_LIBRARY_SUFFIX})

if (NOT EXISTS ${gtest_LIBS})
    externalproject_add(gtest_external
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
            INSTALL_COMMAND ""
            TEST_COMMAND ""
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
endif()
add_library(gtest INTERFACE IMPORTED GLOBAL)
add_dependencies(gtest gtest_external)
file(MAKE_DIRECTORY ${gtest_INCLUDE_DIR})
target_include_directories(gtest INTERFACE ${gtest_INCLUDE_DIR})
target_link_libraries(gtest INTERFACE ${gtest_LIBS})

add_library(gtest_main INTERFACE IMPORTED GLOBAL)
add_dependencies(gtest_main gtest_external)
set(gtest_main_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external/googletest/include)
set(gtest_main_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external-build/lib/libgtest_main${CMAKE_STATIC_LIBRARY_SUFFIX})
file(MAKE_DIRECTORY ${gtest_main_INCLUDE_DIR})
target_include_directories(gtest_main INTERFACE ${gtest_main_INCLUDE_DIR})
target_link_libraries(gtest_main INTERFACE ${gtest_main_LIBS})

add_library(gmock INTERFACE IMPORTED GLOBAL)
add_dependencies(gmock gtest_external)
set(gmock_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external/googlemock/include)
set(gmock_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external-build/lib/libgmock${CMAKE_STATIC_LIBRARY_SUFFIX})
file(MAKE_DIRECTORY ${gmock_INCLUDE_DIR})
target_include_directories(gmock INTERFACE ${gmock_INCLUDE_DIR})
target_link_libraries(gmock INTERFACE ${gmock_LIBS})

add_library(gmock_main INTERFACE IMPORTED GLOBAL)
add_dependencies(gmock_main gtest_external)
set(gmock_main_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external/googlemock/include)
set(gmock_main_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/gtest_external-build/lib/libgmock_main${CMAKE_STATIC_LIBRARY_SUFFIX})
file(MAKE_DIRECTORY ${gmock_main_INCLUDE_DIR})
target_include_directories(gmock_main INTERFACE ${gmock_main_INCLUDE_DIR})
target_link_libraries(gmock_main INTERFACE ${gmock_main_LIBS})
