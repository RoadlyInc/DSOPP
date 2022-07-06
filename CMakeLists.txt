cmake_minimum_required(VERSION 3.11.4)
project(dsopp C CXX)

include(CheckIPOSupported)
check_ipo_supported(RESULT lto_supported OUTPUT lto_error)

if(lto_supported AND LTO_OPTIMIZATION)
  message(STATUS "IPO / LTO enabled")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  if(NOT LTO_OPTIMIZATION)
    set(lto_error " LTO_OPTIMIZATION options set off ")
  endif()
  message(STATUS "IPO / LTO disabled: <${lto_error}>")
endif()

set(CMAKE_CXX_STANDARD 20)

add_definitions(-DPROTOBUF_INLINE_NOT_IN_HEADERS=0)
add_compile_definitions($<$<BOOL:${VISUALIZATION}>:VISUALIZATION>)

include(cmake/options.cmake)
include(cmake/tools.cmake)

include(dependencies.cmake)

set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -pthread")
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -Wconversion -fdiagnostics-color=always"
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 12.0)
    # wouldn't build on g++-12 without this
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized -Wno-uninitialized -Wno-restrict"
    )
  endif()

  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization"
  )
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wformat=2 -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept"
  )
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow"
  )
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wsign-promo -Wsign-conversion -Wstrict-null-sentinel -Wstrict-overflow=2"
  )
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wundef -Wunused-variable -Wno-variadic-macros -Wno-parentheses"
  )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fconcepts-diagnostics-depth=2")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie") # TODO: understand why this
                                                    # flag is necessary

  if(TEST_COVERAGE)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
      message(
        FATAL_ERROR
          "Release build type is not compatible with test coverage build mode")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
  endif()
endif()
if(RUN_CPP_CHECK)
  set(CMAKE_CXX_CPPCHECK "cppcheck")
endif()

add_subdirectory(src)

add_subdirectory(pydsopp)

enable_testing()
include(cmake/addTest.cmake)
add_subdirectory(test)
