#include "sensors/camera_calibration/fabric.hpp"

#include <fstream>
#include <limits>
#include <optional>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "common/file_tools/parsing.hpp"
#include "energy/camera_model/fisheye/atan_camera.hpp"
#include "energy/camera_model/fisheye/tum_fov_model.hpp"
#include "energy/camera_model/pinhole/ios_camera_model.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/undistorter/undistorter.hpp"

namespace dsopp {
namespace sensors {
namespace calibration {
namespace camera_calibration {
namespace {
CameraCalibration createPinholeCameraCalibration(std::ifstream &file, time::duration shutter_time) {
  Eigen::Vector2<Precision> image_size;
  file >> image_size(0) >> image_size(1);

  Eigen::Vector<Precision, 4> intrinsics;
  file >> intrinsics(0) >> intrinsics(1) >> intrinsics(2) >> intrinsics(3);
  return CameraCalibration(image_size, intrinsics, energy::model::ModelType::kPinholeCamera, shutter_time,
                           Undistorter::Identity(image_size));
}

CameraCalibration createSimpleRadialCameraCalibration(std::ifstream &file, time::duration shutter_time,
                                                      bool transform_to_pinhole) {
  Eigen::Vector2<Precision> image_size;
  file >> image_size(0) >> image_size(1);

  Eigen::Vector<Precision, 5> intrinsics;
  file >> intrinsics(0) >> intrinsics(1) >> intrinsics(2) >> intrinsics(3) >> intrinsics(4);

  if (!transform_to_pinhole) {
    return CameraCalibration(image_size, intrinsics, energy::model::ModelType::kSimpleRadialCamera, shutter_time,
                             Undistorter::Identity(image_size));
  } else {
    auto [pinhole, undistorter] =
        Undistorter::constructRemaps(energy::model::SimpleRadialCamera(image_size, intrinsics));

    return CameraCalibration(pinhole.image_size(), pinhole.intrinsicsParameters(),
                             energy::model::ModelType::kPinholeCamera, shutter_time, Undistorter(undistorter));
  }
}

CameraCalibration createTumFovCalibration(std::ifstream &file, time::duration shutter_time) {
  Eigen::Vector2<Precision> image_size;
  file >> image_size(0) >> image_size(1);

  Eigen::Vector2<Precision> focals;
  file >> focals(0) >> focals(1);
  focals[0] *= image_size[0];
  focals[1] *= image_size[1];
  Eigen::Vector2<Precision> center;
  file >> center(0) >> center(1);
  center[0] *= image_size[0];
  center[1] *= image_size[1];
  Precision fov;
  file >> fov;
  auto [pinhole, undistorter] =
      Undistorter::constructRemaps(energy::model::TUMFovModel(image_size, focals, center, fov));

  return CameraCalibration(pinhole.image_size(), pinhole.intrinsicsParameters(),
                           energy::model::ModelType::kPinholeCamera, shutter_time, Undistorter(undistorter));
}

}  // namespace
std::optional<CameraCalibration> create(const std::map<std::string, std::any> &parameters, bool transform_to_pinhole) {
  if (parameters.count("calibration") == 0) {
    LOG(WARNING) << "Missing field \"calibration\" in the model parameters";
    return std::nullopt;
  }

  time::duration shutter_time = time::duration(0);
  if (parameters.count("shutter_time_seconds") != 0) {
    Precision shutter_time_seconds =
        common::file_tools::stringToPrecision(std::any_cast<std::string>(parameters.at("shutter_time_seconds")));
    shutter_time = std::chrono::duration_cast<time::duration>(std::chrono::duration<Precision>(shutter_time_seconds));
  } else {
    LOG(WARNING) << "Shutter time not found in config, setting to " << shutter_time.count();
  }

  std::string fname = std::any_cast<std::string>(parameters.at("calibration"));
  std::ifstream f(fname);
  if (f.is_open()) {
    std::string model_type;
    f >> model_type;

    if (model_type == "pinhole") {
      return createPinholeCameraCalibration(f, shutter_time);
    } else if (model_type == "simple_radial") {
      return createSimpleRadialCameraCalibration(f, shutter_time, transform_to_pinhole);
    } else if (model_type == "tum_fov") {
      if (!transform_to_pinhole) {
        LOG(ERROR) << "Can't create TUM FOV model without undistorter";
        return std::nullopt;
      }
      return createTumFovCalibration(f, shutter_time);
    } else {
      LOG(ERROR) << "Inappropriate model type for camera sensor";
      return std::nullopt;
    }
  } else {
    LOG(ERROR) << "Could not open calibration file " << fname;
    return std::nullopt;
  }
}
}  // namespace camera_calibration
namespace photometric_calibration {

namespace {
PhotometricCalibration defaultPhotometricCalibration() {
  PhotometricCalibration photometric_calibration;
  std::iota(photometric_calibration.begin(), photometric_calibration.end(), 0);
  return photometric_calibration;
}
}  // namespace

PhotometricCalibration create(const std::string &photometric_calibration_file) {
  auto stream = std::ifstream(photometric_calibration_file);
  if (!stream.is_open()) {
    LOG(WARNING) << "File " << photometric_calibration_file << " doesn't exist! The photometric calibration is default";
    return defaultPhotometricCalibration();
  }
  PhotometricCalibration photometric_calibration;
  for (size_t i = 0; i < 256; ++i) {
    stream >> photometric_calibration[i];
  }
  stream.close();
  return PhotometricCalibration(photometric_calibration);
}

PhotometricCalibration create(const std::map<std::string, std::any> &parameters) {
  if (parameters.count("photometric_calibration") == 0) {
    LOG(WARNING) << "Missing field \"photometric_calibration\" in the model parameters";
    return defaultPhotometricCalibration();
  }

  auto photometric_calibration_file = std::any_cast<std::string>(parameters.at("photometric_calibration"));

  return create(photometric_calibration_file);
}
}  // namespace photometric_calibration

namespace vignetting {
cv::Mat create(const std::string &vignetting_path) {
  auto vignetting = cv::imread(vignetting_path, cv::IMREAD_GRAYSCALE);

  if (vignetting.data == nullptr) {
    LOG(WARNING) << "Image " << vignetting_path << " can't be read! The vignetting is default";
    return cv::Mat();
  }

  return vignetting;
}

cv::Mat create(const std::map<std::string, std::any> &parameters) {
  if (parameters.count("vignetting") == 0) {
    LOG(WARNING) << "Missing field \"vignetting\" in the model parameters";
    return cv::Mat();
  }

  auto vignetting_path = std::any_cast<std::string>(parameters.at("vignetting"));

  return create(vignetting_path);
}
}  // namespace vignetting
namespace mask {
std::optional<sensors::calibration::CameraMask> create(const std::map<std::string, std::any> &parameters, int width,
                                                       int height) {
  if (parameters.contains("segmentation")) {
    const auto &segmentation = std::any_cast<std::map<std::string, std::any>>(parameters.at("segmentation"));
    if (segmentation.contains("mask")) {
      cv::Mat cv_mask = cv::imread(std::any_cast<std::string>(segmentation.at("mask")), 0);

      if (cv_mask.empty()) {
        LOG(ERROR) << "Invalid file for mask";
        return std::nullopt;
      } else {
        if (cv_mask.cols != width || cv_mask.rows != height) {
          LOG(ERROR) << "Incompatible mask and calibration sizes" << cv_mask.rows << " " << height;
          return std::nullopt;
        }
        return sensors::calibration::CameraMask(cv_mask);
      }
    }
  }

  LOG(WARNING) << "Mask was not passed, creating empty mask";

  return sensors::calibration::CameraMask(height, width);
}
}  // namespace mask
}  // namespace calibration
}  // namespace sensors
}  // namespace dsopp
