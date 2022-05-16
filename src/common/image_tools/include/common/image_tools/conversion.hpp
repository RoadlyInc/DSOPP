#ifndef DSOPP_COMMON_IMAGE_TOOLS_CONVERSION_HPP_
#define DSOPP_COMMON_IMAGE_TOOLS_CONVERSION_HPP_

#include "opencv2/core.hpp"

#include "features/camera/pixel_map.hpp"

namespace dsopp::common::image_tools {

/**
 * Converts PixelMap given channel image to cv::Mat grayscale image
 *
 * @tparam ChannelsNumber PixelMap image channels number
 * @param pixel_map PixelMap image
 * @param channel_index given channel index
 * @return cv::Mat grayscale image
 */
template <int ChannelsNumber>
cv::Mat pixelMap2Mat1C(const features::PixelMap<ChannelsNumber> &pixel_map, int channel_index = 0);

/**
 * Converts PixelMap first 3 channels image to cv::Mat image
 *
 * @tparam ChannelsNumber PixelMap image channels number
 * @param pixel_map PixelMap image
 * @return cv::Mat image
 */
template <int ChannelsNumber>
cv::Mat pixelMap2Mat3C(const features::PixelMap<ChannelsNumber> &pixel_map);

/**
 * Converts PixelMap first 3 channels image to cv::Mat image using PCA transformation
 *
 * @tparam ChannelsNumber PixelMap image channels number
 * @param pixel_map PixelMap image
 * @param pca pca transformation. It takes three main components from the kChannelsNumber
 * @return cv::Mat image
 */
template <int ChannelsNumber>
cv::Mat pixelMap2Mat3C(const features::PixelMap<ChannelsNumber> &pixel_map,
                       const Eigen::Matrix<Precision, 3, ChannelsNumber> &pca);

}  // namespace dsopp::common::image_tools

#endif  // DSOPP_COMMON_IMAGE_TOOLS_CONVERSION_HPP_
