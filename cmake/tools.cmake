##########################
#        Doxygen         #
##########################
if (BUILD_DOC)
    find_package(Doxygen
            REQUIRED dot
            OPTIONAL_COMPONENTS mscgen dia)

    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    add_custom_target(doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
endif()

##############################
#   clang and cmake format   #
##############################
if (CHECK_FORMAT)
    add_custom_target(
            check_format ALL
            COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_clang_format.py -r ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/test ${CMAKE_CURRENT_SOURCE_DIR}/pydsopp
            COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/src/ -name CMakeLists.txt | xargs cmake-format --check
            COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/test/ -name CMakeLists.txt | xargs cmake-format --check
            COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/pydsopp/ -name CMakeLists.txt | xargs cmake-format --check
            COMMAND cmake-format --check ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
            COMMAND yapf -d -r ${CMAKE_CURRENT_SOURCE_DIR}/test/pydsopp
            COMMAND yapf -d -r ${CMAKE_CURRENT_SOURCE_DIR}/pydsopp
    )
endif()


##########################
#   dependencies check   #
##########################
if (DEPENDENCIES_CHECK)
    add_custom_target(
            dependencies_graph ALL
            COMMAND cmake --graphviz=dependencies.dot . ${CMAKE_CURRENT_SOURCE_DIR}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Building dependencies graph"
            VERBATIM
    )
    add_custom_target(
            check_dependencies ALL
            COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_circular_dependencies.py
            ${CMAKE_CURRENT_BINARY_DIR}/dependencies.dot
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS dependencies_graph
            COMMENT "checking dependencies graph"
            VERBATIM
    )
endif()

#############################
#    Compile hidden code    #
#############################
if (COMPILE_HIDDEN_CODE)
    add_compile_definitions(COMPILE_HIDDEN_CODE=true)
else()
    add_compile_definitions(COMPILE_HIDDEN_CODE=false)
endif()

#############################
#    use float precision    #
#############################
if (USE_FLOAT)
    add_compile_definitions(USE_FLOAT=true)
else()
    add_compile_definitions(USE_FLOAT=false)
endif()