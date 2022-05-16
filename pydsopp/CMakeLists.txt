add_subdirectory(features)

pybind11_add_module(pydsopp pydsopp.cpp)
target_link_libraries(pydsopp PRIVATE features_wrapper)

set(PY_SOPHUS "${sophus_INCLUDE_DIR}/py")
get_target_property(STORAGE_PATH storage BINARY_DIR)

set(LIST_OF_CONFIGURED_FILES transformations/__sophus_import.py
                             storage/track_storage.py)
set(LIST_OF_COPYED_FILES
    utils/extract_images.py
    utils/point_cloud_exporter.py
    utils/track2json.py
    utils/track2colmap.py
    track/track.py
    transformations/utils.py
    transformations/__init__.py
    transformations/motion.py
    track/camera_calibration.py
    track/frame.py
    track/__init__.py
    track/export/colmap_track_export.py
    track/export/json_track_export.py)

add_custom_target(pydsopp_lib ALL DEPENDS proto_python sophus
                                          ${LIST_OF_CONFIGURED_FILES})

foreach(file ${LIST_OF_CONFIGURED_FILES})
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${file}"
                 "${CMAKE_CURRENT_BINARY_DIR}/${file}")
endforeach()

foreach(file ${LIST_OF_COPYED_FILES})
  add_custom_command(
    TARGET pydsopp_lib
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/${file}"
            "${CMAKE_CURRENT_BINARY_DIR}/${file}")
endforeach()
