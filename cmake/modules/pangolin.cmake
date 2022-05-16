set(URL https://github.com/stevenlovegrove/Pangolin.git)
set(VERSION 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d)
set(pangolin_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/pangolin_external-build/install/include)
set(pangolin_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/pangolin_external-build/install/lib/libpangolin${CMAKE_SHARED_LIBRARY_SUFFIX})
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

if (NOT EXISTS ${pangolin_LIBS})
    externalproject_add(pangolin_external
            DEPENDS eigen
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
                -DEIGEN_INCLUDE_DIR:PATH=${eigen_INCLUDE_DIRS}
                -DCMAKE_INSTALL_PREFIX=./install
        TEST_COMMAND ""
        PREFIX 3rd_party
        EXCLUDE_FROM_ALL 1
        )
endif()
add_library(pangolin INTERFACE IMPORTED GLOBAL)
add_dependencies(pangolin pangolin_external)
file(MAKE_DIRECTORY ${pangolin_INCLUDE_DIR})
target_include_directories(pangolin INTERFACE ${pangolin_INCLUDE_DIR} )
target_link_libraries(pangolin INTERFACE ${OPENGL_LIBRARIES})
target_link_libraries(pangolin INTERFACE ${pangolin_LIBS} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
