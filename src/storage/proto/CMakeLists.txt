file(
  GLOB PROTO_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  "*.proto")
file(GLOB PROTO_FILES_DEPENDENCIES "${CMAKE_CURRENT_SOURCE_DIR}/schema/*.proto")

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
protobuf_generate_python(PROTO_PY ${PROTO_FILES})

add_library(proto_storage ${PROTO_SRCS} ${PROTO_HDRS})
set_target_properties(
  proto_storage
  PROPERTIES
    COMPILE_FLAGS
    "-Wno-sign-conversion -Wno-unused-parameter -Wno-redundant-decls -Wno-conversion"
)

add_custom_target(proto_python ALL DEPENDS ${PROTO_PY})

target_link_libraries(proto_storage ${PROTOBUF_LIBRARIES})
target_include_directories(proto_storage INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
