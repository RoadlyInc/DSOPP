set(URL https://github.com/pybind/pybind11.git)
set(VERSION v2.7.1)

include(FetchContent)
FetchContent_Declare(pybind11
        GIT_REPOSITORY ${URL}
        GIT_TAG        ${VERSION})

FetchContent_GetProperties(pybind11)
if(NOT pybind11_POPULATED)
    FetchContent_Populate(pybind11)
    add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
endif()

add_library(pybind11 INTERFACE IMPORTED GLOBAL)
target_include_directories(pybind11 INTERFACE ${pybind11_INCLUDE_DIRS})
