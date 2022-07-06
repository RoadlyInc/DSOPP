set(URL  https://ceres-solver.googlesource.com/ceres-solver)
set(VERSION 5cb5b35a930c1702278083c75769dbb4e5801045)
set(ceres_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/ceres_external-build/install/include)
set(ceres_LIBS ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/ceres_external-build/install/lib/libceres${CMAKE_STATIC_LIBRARY_SUFFIX})

find_package(LAPACK QUIET)
find_package(benchmark QUIET)
find_package(Sphinx QUIET)

get_target_property(glog_INCLUDE_DIRS glog INTERFACE_INCLUDE_DIRECTORIES)
get_property(glog_lib GLOBAL PROPERTY glog_path)
set(Glog_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/glog_external-build/install/lib/cmake/glog/
)

get_target_property(eigen_INCLUDE_DIRS eigen INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(eigen_cmake "${eigen_INCLUDE_DIRS}/../../share/eigen3/cmake" ABSOLUTE)

set(template_specialization_file_dir
        ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/ceres_external/internal/ceres/
        )
set(ceres_find_cmake_dir
        ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/ceres_external/cmake/
        )

set(template_specialization_file
        generate_template_specializations.py
        )

if (NOT EXISTS ${ceres_LIBS})
    externalproject_add(ceres_external
            DEPENDS glog eigen
            GIT_REPOSITORY ${URL}
            GIT_TAG        ${VERSION}
            CMAKE_ARGS
                -DCMAKE_BUILD_TYPE=Release
                -DCERES_THREADING_MODEL=CXX_THREADS
                -DCMAKE_CXX_STANDARD=20
                -DCMAKE_CXX_STANDARD_REQUIRED=ON
                -DCMAKE_CXX_FLAGS=-march=native
                -DCMAKE_INSTALL_PREFIX=./install
                -DBUILD_TESTING=OFF
                -DCXSPARSE=OFF
                -DBUILD_BENCHMARKS=OFF
                -DGFLAGS=OFF
                -DGLOG_INCLUDE_DIR=${glog_INCLUDE_DIRS}
                -DGLOG_LIBRARY=${glog_lib}
                -Dglog_DIR=${Glog_DIR}
                -DBUILD_EXAMPLES=OFF
                -DEigen3_DIR=${eigen_cmake}
            TEST_COMMAND ""
            PATCH_COMMAND
                sed -i -e "s|(4, 4, \"Eigen::Dynamic\")|(8, 1, 8)|" ${template_specialization_file_dir}${template_specialization_file} &&
                cd ${template_specialization_file_dir} && python2 ${template_specialization_file} &&
                cd ${ceres_find_cmake_dir} && rm FindTBB.cmake && wget https://raw.githubusercontent.com/ceres-solver/ceres-solver/master/cmake/FindTBB.cmake
            PREFIX 3rd_party
            EXCLUDE_FROM_ALL 1
            )
endif()
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
find_package(SuiteSparse REQUIRED)
add_library(ceres INTERFACE IMPORTED GLOBAL)
add_dependencies(ceres ceres_external)
file(MAKE_DIRECTORY ${ceres_INCLUDE_DIR})
target_include_directories(ceres INTERFACE ${ceres_INCLUDE_DIR} ${eigen_INCLUDE_DIR})
target_link_libraries(ceres INTERFACE ${ceres_LIBS} glog ${SUITESPARSE_LIBRARIES})
