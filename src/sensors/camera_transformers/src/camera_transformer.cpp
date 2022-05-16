#include "sensors/camera_transformers/camera_transformer.hpp"

namespace dsopp::sensors::camera_transformers {

cv::Mat runImageTransformers(const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers,
                             const cv::Mat &image) {
  cv::Mat dst = image;
  for (auto &transformer : transformers) {
    transformer->transformImage(dst);
  }
  return dst;
}

cv::Mat runMaskTransformers(const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers,
                            const cv::Mat &mask) {
  cv::Mat dst = mask;
  for (auto &transformer : transformers) {
    transformer->transformMask(dst);
  }
  return dst;
}

}  // namespace dsopp::sensors::camera_transformers
