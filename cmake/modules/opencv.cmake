set(CONTRIB_URL https://github.com/opencv/opencv_contrib.git)
set(CONTRIB_VERSION 3b5a55876fe0502418a9d9fb7d388c40f2a626b1)

set(contrib_INCLUDE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/opencv_contrib_external/)
set(contrib_LIBS
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/opencv_contrib_external/modules)

if(NOT EXISTS ${contrib_LIBS})
  ExternalProject_Add(
    opencv_contrib_external
    GIT_REPOSITORY ${CONTRIB_URL}
    GIT_TAG ${CONTRIB_VERSION}
    CONFIGURE_COMMAND ""
    CMAKE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    TEST_COMMAND ""
    PREFIX 3rd_party
    EXCLUDE_FROM_ALL 1)
  file(MAKE_DIRECTORY ${contrib_INCLUDE_DIR})
endif()

add_library(opencv_contrib INTERFACE IMPORTED GLOBAL)
add_dependencies(opencv_contrib opencv_contrib_external)

set(URL https://github.com/opencv/opencv.git)
set(VERSION 9aa647068b2eba4a34462927b1878353dfd3df69)

set(opencv_INCLUDE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/opencv_external-build/install/include/opencv4
)
set(opencv_LIBS
    ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/opencv_external-build/install/lib)

get_target_property(glog_INCLUDE_DIRS glog INTERFACE_INCLUDE_DIRECTORIES)
get_property(glog_lib GLOBAL PROPERTY glog_path)

if(NOT EXISTS ${opencv_LIBS})
  ExternalProject_Add(
    opencv_external
    DEPENDS opencv_contrib glog
    GIT_REPOSITORY ${URL}
    GIT_TAG ${VERSION}
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
               -DINSTALL_C_EXAMPLES=OFF
               -DINSTALL_PYTHON_EXAMPLES=OFF
               -DOPENCV_GENERATE_PKGCONFIG=OFF
               -DOPENCV_EXTRA_MODULES_PATH=${contrib_LIBS}
               -DOPENCV_ENABLE_NONFREE=ON
               -DCMAKE_CXX_STANDARD=20
               -DCMAKE_CXX_STANDARD_REQUIRED=ON
               -DCMAKE_INSTALL_PREFIX=./install
               -DGLOG_INCLUDE_DIR=${glog_INCLUDE_DIRS}
               -DGLOG_LIBRARY=${glog_lib}
               -DWITH_FFMPEG=ON
               -DBUILD_PERF_TESTS=OFF
               -DBUILD_TESTS=OFF
               -DBUILD_opencv_xphoto=OFF
               -DBUILD_opencv_datasets=OFF
               -DBUILD_opencv_rgbd=OFF
               -DBUILD_opencv_barcode=OFF
               -DBUILD_opencv_xobjdetect=OFF
               -DBUILD_opencv_aruco=OFF
               -DBUILD_opencv_wechat_qrcode=OFF
               -DBUILD_opencv_python3=OFF
               -DBUILD_opencv_python_bindings_generator=OFF
               -DBUILD_opencv_python_tests=OFF
               -DBUILD_opencv_structured_light=OFF
               -DBUILD_opencv_dnn=OFF
               -DBUILD_opencv_dnn_objdetect=OFF
               -DBUILD_opencv_dnn_superres=OFF
               -DBUILD_opencv_face=OFF
               -DBUILD_opencv_bioinspired=OFF
               -DBUILD_opencv_phase_unwrapping=OFF
               -DBUILD_opencv_line_descriptor=OFF
    TEST_COMMAND ""
    PREFIX 3rd_party
    EXCLUDE_FROM_ALL 1)
endif()

add_library(opencv INTERFACE IMPORTED GLOBAL)
add_dependencies(opencv opencv_external)
file(MAKE_DIRECTORY ${opencv_INCLUDE_DIR})
target_include_directories(opencv INTERFACE ${opencv_INCLUDE_DIR})

set(OPENCV_LIBS)

foreach(
  library
  opencv_core
  opencv_flann
  opencv_imgproc
  opencv_intensity_transform
  opencv_ml
  opencv_photo
  opencv_plot
  opencv_quality
  opencv_reg
  opencv_surface_matching
  opencv_alphamat
  opencv_features2d
  opencv_freetype
  opencv_fuzzy
  opencv_hfs
  opencv_img_hash
  opencv_imgcodecs
  opencv_saliency
  opencv_videoio
  opencv_calib3d
  opencv_highgui
  opencv_objdetect
  opencv_rapid
  opencv_shape
  opencv_video
  opencv_videostab
  opencv_xfeatures2d
  opencv_ximgproc
  opencv_bgsegm
  opencv_ccalib
  opencv_dpm
  opencv_gapi
  opencv_optflow
  opencv_sfm
  opencv_stitching
  opencv_superres
  opencv_tracking
  opencv_stereo)
  list(APPEND OPENCV_LIBS
       ${opencv_LIBS}/lib${library}${CMAKE_SHARED_LIBRARY_SUFFIX})
endforeach()

target_link_libraries(opencv INTERFACE ${OPENCV_LIBS})
