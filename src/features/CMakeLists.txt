add_library(
  features
  src/camera_features.cpp
  src/pattern_patch.cpp
  src/photometrically_corrected_image.cpp
  src/pixel_data_frame.cpp
  src/pixel_data_frame_extractor.cpp
  src/pixel_map.cpp
  src/calculate_pixelinfo.cpp
  src/tracking_feature.cpp
  src/eigen_tracking_features_extractor.cpp
  src/sobel_tracking_features_extractor.cpp
  src/tracking_features_frame.cpp
  #[[ contact Roadly INC for this functionality ]]
  #[[
  src/gn_net_extractor.cpp  GN-Net: The Gauss-Newton Loss for Multi-Weather Relocalization (L. von Stumberg, P. Wenzel, Q. Khan and D. Cremers),
  src/identity_extractor.cpp
  src/gnss_features.cpp
  src/imu_features.cpp
  src/btf_extractor.cpp  Back to the Feature: Learning Robust Camera Localization from Pixels to Pose Paul-Edouard Sarlin, Ajaykumar Unagar, Måns Larsson, Hugo Germain, Carl Toft, Viktor Larsson, Marc Pollefeys, Vincent Lepetit, Lars Hammarstrand, Fredrik Kahl, Torsten Sattler
  ]]
)

target_include_directories(
  features PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include/dsopp>)

target_include_directories(
  features PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal/>)

target_link_libraries(
  features
  PUBLIC camera_calibration
         time
         common_constexpr_tools
         common_pattern
         common_patch
         settings
         common_semantics)
target_link_libraries(features PUBLIC opencv ceres)

target_link_libraries(features PRIVATE glog stdc++fs tbb)
