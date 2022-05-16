#include "sensors/camera_transformers/fabric.hpp"

#include <glog/logging.h>

#include "common/file_tools/parsing.hpp"
#include "common/settings.hpp"
#include "sensors/camera_transformers/camera_resizer.hpp"
#include "sensors/camera_transformers/image_cropper.hpp"

namespace dsopp::sensors::camera_transformers {

std::vector<std::unique_ptr<CameraTransformer>> create(const std::map<std::string, std::any> &parameters) {
  std::vector<std::unique_ptr<CameraTransformer>> transformers;

  if (parameters.contains("resize_transformer")) {
    const auto &resize_transformer_parameters =
        std::any_cast<std::map<std::string, std::any>>(parameters.at("resize_transformer"));

    if (resize_transformer_parameters.contains("resize_ratio")) {
      Precision new_to_old_size_ratio = common::file_tools::stringToPrecision(
          std::any_cast<std::string>(resize_transformer_parameters.at("resize_ratio")));
      transformers.emplace_back(std::make_unique<camera_transformers::CameraResizer>(new_to_old_size_ratio));
      LOG(INFO) << "Created camera resizer";
    } else {
      LOG(ERROR) << "Could not create image resizer \"resize_ratio\" field does not exist";
    }
  }
  transformers.emplace_back(std::make_unique<camera_transformers::ImageCropper>());

  return transformers;
}
}  // namespace dsopp::sensors::camera_transformers
