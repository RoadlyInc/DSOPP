#ifndef DSOPP_FRAME_EMBEDDING_EXTRACTOR_HPP
#define DSOPP_FRAME_EMBEDDING_EXTRACTOR_HPP

#include <array>
#include <memory>

#include <opencv2/core.hpp>

#include "features/camera/pixel_map.hpp"

namespace dsopp::features {
/**
 * \brief Base class for frame embedding extractors
 *
 * @tparam outputChannels output channels number
 */
template <int outputChannels>
class FrameEmbeddingExtractor {
 public:
  /** number of pyramids in neural network model */
  static constexpr int kPyramidLevels = 4;
  /** shift scale for evaluate resize by levels*/
  static constexpr int kLevelShiftScale = 1;
  /** numbers of channels on the pyramid levels */
  static constexpr std::array<int, kPyramidLevels> kChannels = {outputChannels, outputChannels, outputChannels,
                                                                outputChannels};
  /** values of level resize */
  static constexpr std::array<int, kPyramidLevels> kResizeValues = {1, 2, 4, 8};

  /**
   * extracts frame embedding from image
   *
   * @param image image
   * @return frame embedding
   */
  virtual std::unique_ptr<PixelMap<outputChannels>> extract(const cv::Mat &image) = 0;

  /**
   * extracts deep pyramids from image
   *
   * @param image image
   * @return unique pointer to vector of pyramids
   */
  virtual std::vector<PixelMap<outputChannels>> extractPyramids(const cv::Mat &image) = 0;

  /**
   * @return PCA transformation for the image after neural network. It takes three main components from the
   * kChannelsNumber
   */
  const Eigen::Matrix<Precision, 3, outputChannels> &pca() { return pca_; }

  virtual ~FrameEmbeddingExtractor() = default;

 protected:
  /** PCA transformation for the image after neural network. It takes three main components from the kChannelsNumber */
  Eigen::Matrix<Precision, 3, outputChannels> pca_ = Eigen::Matrix<Precision, 3, outputChannels>::Identity();
};
}  // namespace dsopp::features

#endif  // DSOPP_FRAME_EMBEDDING_EXTRACTOR_HPP
