#ifndef DSOPP_PHOTOMETRICALLY_CORRECTED_IMAGE_HPP
#define DSOPP_PHOTOMETRICALLY_CORRECTED_IMAGE_HPP

#include <array>

#include "opencv2/core.hpp"

#include "common/settings.hpp"

namespace dsopp::features {
/**
 * Photometrically corrects grayscale uchar image
 *
 * @param image grayscale uchar image
 * @param photometric_calibration photometric calibration
 * @param vignetting vignetting
 * @return photometrically corrected grayscale Precision image
 */
std::vector<Precision> photometricallyCorrectedImage(const cv::Mat &image,
                                                     const std::array<Precision, 256> &photometric_calibration,
                                                     const cv::Mat &vignetting);
}  // namespace dsopp::features

#endif  // DSOPP_PHOTOMETRICALLY_CORRECTED_IMAGE_HPP
