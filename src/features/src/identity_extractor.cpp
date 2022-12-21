#include "features/camera/identity_extractor.hpp"

#include <glog/logging.h>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "features/camera/downscale_image.hpp"
#include "features/camera/photometrically_corrected_image.hpp"

namespace dsopp::features {

namespace {

/**
 * creatre std vector with ``output_channels`` copies of image
 */
std::vector<Precision, PrecisionAllocator> copyChannels(std::vector<Precision, PrecisionAllocator> input,
                                                        size_t output_channels) {
  std::vector<Precision, PrecisionAllocator> output(output_channels * input.size());
  for (size_t c = 0; c < output_channels; ++c) {
    std::memcpy(output.data() + c * input.size(), input.data(), input.size() * sizeof(Precision));
  }
  return output;
}

}  // namespace

template <int outputChannels>
IdentityExtractor<outputChannels>::IdentityExtractor(const std::array<Precision, 256> &photometric_calibration,
                                                     const cv::Mat &undistorted_vignetting)
    : photometric_calibration_(photometric_calibration), undistorted_vignetting_(undistorted_vignetting) {}

template <int outputChannels>
std::unique_ptr<PixelMap<outputChannels>> IdentityExtractor<outputChannels>::extract(const cv::Mat &image) {
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  std::vector<Precision, PrecisionAllocator> photometrically_corrected_image =
      photometricallyCorrectedImage(image_gray, this->photometric_calibration_, this->undistorted_vignetting_);

  auto frame_embedding_data = copyChannels(photometrically_corrected_image, static_cast<size_t>(outputChannels));

  return std::make_unique<PixelMap<outputChannels>>(std::move(frame_embedding_data), static_cast<size_t>(image.cols),
                                                    static_cast<size_t>(image.rows));
}

template <int outputChannels>
std::vector<PixelMap<outputChannels>> IdentityExtractor<outputChannels>::extractPyramids(const cv::Mat &image) {
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  std::vector<Precision, PrecisionAllocator> curr_image =
      photometricallyCorrectedImage(image_gray, this->photometric_calibration_, this->undistorted_vignetting_);

  std::vector<PixelMap<outputChannels>> out;

  auto width = static_cast<int>(image.cols);
  auto height = static_cast<int>(image.rows);

  for (size_t lvl = 0; lvl < this->kMaxPyramidLevels; ++lvl) {
    auto frame_embedding_data = copyChannels(curr_image, static_cast<size_t>(outputChannels));
    out.emplace_back(std::move(frame_embedding_data), width, height);

    curr_image = downscaleImage(curr_image, height, width);

    width /= 2;
    height /= 2;
  }

  return out;
}

template class IdentityExtractor<kChannelsNumber>;
}  // namespace dsopp::features
