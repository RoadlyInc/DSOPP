set(URL https://github.com/stevenlovegrove/Pangolin.git)
set(VERSION 4a9b2b013c2651377d74a7458b15c5affd2d232c)
set(pangolin_INCLUDE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/pangolin_external-build/install/include
)

set(pangolin_LIBS
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/pangolin_external-build/install/lib/
)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

if(NOT EXISTS ${pangolin_LIBS})
  ExternalProject_Add(
    pangolin_external
    DEPENDS eigen
    GIT_REPOSITORY ${URL}
    GIT_TAG ${VERSION}
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
               -DEIGEN_INCLUDE_DIR:PATH=${eigen_INCLUDE_DIRS}
               -DCMAKE_INSTALL_PREFIX=./install
    TEST_COMMAND ""
    PREFIX 3rd_party
    EXCLUDE_FROM_ALL 1)
endif()
add_library(pangolin INTERFACE IMPORTED GLOBAL)
add_dependencies(pangolin pangolin_external)
file(MAKE_DIRECTORY ${pangolin_INCLUDE_DIR})
target_include_directories(pangolin INTERFACE ${pangolin_INCLUDE_DIR})
if(EXISTS ${pangolin_LIBS})
  target_include_directories(pangolin INTERFACE ${pangolin_LIBS})
endif()
set(pangolin_link_LIBS)
foreach(
  library
  pango_vars
  pango_scene
  tinyobj
  pango_windowing
  pango_packetstream
  pango_geometry
  pango_core
  pango_python
  pango_opengl
  pango_tools
  pango_glgeometry
  pango_video
  pango_display
  pango_plot
  pango_image)
  list(
    APPEND
    pangolin_link_LIBS
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/pangolin_external-build/lib${library}${CMAKE_SHARED_LIBRARY_SUFFIX}
  )
endforeach()

target_link_libraries(pangolin INTERFACE ${pangolin_link_LIBS}
                                         ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
