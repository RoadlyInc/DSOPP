#ifndef DSOPP_SRC_FEATURES_EXTRACTION_TOOLS_HPP
#define DSOPP_SRC_FEATURES_EXTRACTION_TOOLS_HPP

#ifdef BUILD_WITH_LIBTORCH
#include <ATen/core/ivalue.h>
#include <torch/script.h>

#include <opencv2/core.hpp>

#include "features/camera/pixel_map.hpp"

namespace dsopp::features {
/**
 * Run ``model`` on ``img_initial`` and output tuple of elements from the extractor
 * @param img_initial image
 * @param model model
 * @return vector of NN outputs
 */
std::vector<c10::IValue> runExtractor(const cv::Mat &img_initial, torch::jit::script::Module &model);

/**
 * Flatten tensor and put it into ``Precision`` vector
 * @param model_output tensor output of the model
 * @param channels_number number of expected channels
 * @return ``Precision`` vector
 */
std::vector<Precision> flatten(torch::Tensor &model_output, int channels_number);

}  // namespace dsopp::features

#endif
#endif
