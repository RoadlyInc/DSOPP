#include "sensors/camera_providers/fabric.hpp"

#include <glog/logging.h>

#include "sensors/camera_providers/image_folder_provider.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "sensors/camera_providers/npy_folder_provider.hpp"

namespace dsopp {
namespace sensors {
namespace providers {

namespace {
size_t readUnnecessaryUint(const std::map<std::string, std::any> &parameters, const std::string &field,
                           size_t default_value) {
  if (!parameters.contains(field)) {
    LOG(WARNING) << "Missing field \"" + field +
                        "\" in parameters. The value of the batch is "
                        "accepted " +
                        std::to_string(default_value);
    return default_value;
  }
  return std::stoul(std::any_cast<std::string>(parameters.at(field)));
}
}  // namespace

std::unique_ptr<CameraProvider> createCameraProvider(const std::map<std::string, std::any> &parameters,
                                                     bool read_grayscale) {
  if (parameters.count("provider") == 0) {
    LOG(WARNING) << "Missing field \"provider\" in the sensor parameters";
    return nullptr;
  }
  const auto &provider = std::any_cast<std::map<std::string, std::any>>(parameters.at("provider"));

  if (provider.count("type") == 0) {
    LOG(WARNING) << "Missing field \"type\" in the sensor parameters";
    return nullptr;
  }
  const auto &provider_type = std::any_cast<std::string>(provider.at("type"));
  auto start_frame = readUnnecessaryUint(provider, "start_frame", 0);
  auto end_frame = readUnnecessaryUint(provider, "end_frame", std::numeric_limits<size_t>::max());
  if (start_frame >= end_frame) {
    LOG(ERROR) << "End frame have to be greater then the start frame";
    return nullptr;
  }

  if (provider_type == "image_folder") {
    if (provider.count("folder") == 0) {
      LOG(WARNING) << "Missing field \"folder\" in the image_folder provider";
      return nullptr;
    }
    if (provider.count("timestamps") == 0) {
      LOG(WARNING) << "Missing field \"timestamps\" in the image_folder provider";
      return nullptr;
    }
    auto batch_size = readUnnecessaryUint(provider, "batch_size", 10);
    const auto &path = std::any_cast<std::string>(provider.at("folder"));
    const auto &timestamps_file = std::any_cast<std::string>(provider.at("timestamps"));
    return std::make_unique<ImageFolderProvider>(path, timestamps_file, batch_size, start_frame, end_frame,
                                                 read_grayscale);
  } else if (provider_type == "video") {
    if (provider.count("video_file") == 0) {
      LOG(WARNING) << "Missing field \"video_file\" in the image video provider";
      return nullptr;
    }
    if (provider.count("timestamps") == 0) {
      LOG(WARNING) << "Missing field \"timestamps\" in the image video provider";
      return nullptr;
    }
    size_t timestamps_frame_id_start = 0;
    if (provider.count("timestamps_start_id") == 0) {
      LOG(WARNING) << "Missing field \"timestamps_start_id\" in the image video provider. Setting as 0.";
    } else {
      timestamps_frame_id_start = std::stoul(std::any_cast<std::string>(provider.at("timestamps_start_id")));
    }

    const auto &path = std::any_cast<std::string>(provider.at("video_file"));
    const auto &timestamps_file = std::any_cast<std::string>(provider.at("timestamps"));
    return std::make_unique<ImageVideoProvider>(path, timestamps_file, start_frame, end_frame,
                                                timestamps_frame_id_start, read_grayscale);
  } else if (provider_type == "npy") {
    if (!provider.contains("folder")) {
      LOG(WARNING) << "Missing field \"folder\" in the npy folder provider";
      return nullptr;
    }
    if (!provider.contains("timestamps")) {
      LOG(WARNING) << "Missing field \"timestamps\" in the npy folder provider";
      return nullptr;
    }
    size_t timestamps_frame_id_start = 0;
    if (!provider.contains("timestamps_start_id")) {
      LOG(WARNING) << "Missing field \"timestamps_start_id\" in the npy folder provider. Setting as 0.";
    } else {
      timestamps_frame_id_start = std::stoul(std::any_cast<std::string>(provider.at("timestamps_start_id")));
    }

    const auto &path = std::any_cast<std::string>(provider.at("folder"));
    const auto &timestamps_file = std::any_cast<std::string>(provider.at("timestamps"));
    return std::make_unique<NpyFolderProvider>(path, timestamps_file, start_frame, end_frame,
                                               timestamps_frame_id_start);
  } else {
    LOG(WARNING) << "Inappropriate provider type for camera sensor";
    return nullptr;
  }
}
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
