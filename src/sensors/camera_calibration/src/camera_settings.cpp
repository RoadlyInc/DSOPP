#include "sensors/camera_calibration/camera_settings.hpp"

#include <glog/logging.h>
#include <memory>
#include <opencv2/imgcodecs.hpp>

namespace dsopp::sensors::calibration {

namespace {
std::string encodeImage(const cv::Mat &image, const std::string &write_extenstion) {
  std::vector<uchar> buffer;
  cv::imencode(write_extenstion, image, buffer);
  return std::string(reinterpret_cast<const char *>(&buffer[0]), buffer.size());
}
cv::Mat decodeImage(const std::string &buffer_str) {
  std::vector<uchar> buffer(buffer_str.begin(), buffer_str.end());
  if (buffer.empty()) return cv::Mat();
  return cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
}
}  // namespace

CameraSettings::CameraSettings(CameraCalibration &&calibration, PhotometricCalibration &&photometric_calibration,
                               Vignetting &&vignetting, calibration::CameraMask &&camera_mask,
                               std::unique_ptr<semantics::SemanticLegend> &&semantic_legend)
    : calibration_(std::move(calibration)),
      photometric_calibration_(std::move(photometric_calibration)),
      vignetting_(std::move(vignetting)),
      camera_mask_(std::move(camera_mask)),
      semantic_legend_(std::move(semantic_legend)) {}

CameraSettings::CameraSettings(const CameraSettings &other)
    : calibration_(other.calibration_.clone()),
      photometric_calibration_(other.photometric_calibration_),
      vignetting_(other.vignetting_),
      camera_mask_(other.camera_mask_),
      semantic_legend_(other.semantic_legend_ ? std::make_unique<semantics::SemanticLegend>(*other.semantic_legend_)
                                              : nullptr) {}
CameraSettings::CameraSettings(CameraSettings &&) = default;
CameraSettings CameraSettings::clone() const { return *this; }
namespace {
CameraCalibration readCalibration(const proto::CameraSettings &camera_settings) {
  auto type = energy::model::ModelType(camera_settings.model_type());
  Eigen::VectorX<Precision> camera_intrinsics(camera_settings.intrinsics_size());
  for (int i = 0; i < camera_settings.intrinsics_size(); ++i) {
    camera_intrinsics(i) = static_cast<Precision>(camera_settings.intrinsics(i));
  }
  Eigen::Vector2<Precision> image_size;
  if (camera_settings.image_size_size() != 2) {
    LOG(ERROR) << "Could not read proto, image_size size is invalid";
  }
  for (int i = 0; i < 2; ++i) {
    image_size(i) = static_cast<Precision>(camera_settings.image_size(i));
  }
  auto shutter_time =
      std::chrono::duration_cast<time::duration>(std::chrono::duration<Precision>(camera_settings.shutter_time()));
  return CameraCalibration(image_size, camera_intrinsics, type, shutter_time, Undistorter::Identity(image_size));
}
}  // namespace
CameraSettings::CameraSettings(const proto::CameraSettings &camera_settings)
    : calibration_(readCalibration(camera_settings)),
      vignetting_(decodeImage(camera_settings.vignetting())),
      camera_mask_(decodeImage(camera_settings.camera_mask())),
      semantic_legend_(std::make_unique<semantics::SemanticLegend>(camera_settings.semantic_legend())) {
  // read photometric calibration
  if (photometric_calibration_.max_size() != static_cast<size_t>(camera_settings.photometric_size())) {
    LOG(ERROR) << "Could not read proto, photometric size is invalid";
  }
  for (size_t i = 0; i < photometric_calibration_.size(); ++i) {
    photometric_calibration_[i] = static_cast<Precision>(camera_settings.photometric(static_cast<int>(i)));
  }
}

proto::CameraSettings CameraSettings::proto() const {
  proto::CameraSettings camera_settings;
  const auto &camera_intrinsics = calibration_.cameraIntrinsics();
  for (int i = 0; i < camera_intrinsics.rows(); ++i) {
    camera_settings.add_intrinsics(camera_intrinsics(i));
  }
  const auto &image_size = calibration_.image_size();
  for (int i = 0; i < image_size.rows(); ++i) {
    camera_settings.add_image_size(image_size(i));
  }
  camera_settings.set_shutter_time(std::chrono::duration<Precision>(calibration_.shutterTime()).count());
  camera_settings.set_model_type(static_cast<proto::CameraSettings_ModelType>(calibration_.type()));

  for (size_t i = 0; i < photometric_calibration_.size(); ++i) {
    camera_settings.add_photometric(photometric_calibration_.at(i));
  }

  const std::string kWriteExtension = ".png";
  if (!vignetting_.empty()) {
    *camera_settings.mutable_vignetting() = encodeImage(vignetting_, kWriteExtension);
  }
  auto data = camera_mask_.data();
  if (!data.empty()) {
    *camera_settings.mutable_camera_mask() = encodeImage(data, kWriteExtension);
  }

  if (semantic_legend_) {
    *camera_settings.mutable_semantic_legend() = semantic_legend_->proto();
  }

  return camera_settings;
}

const semantics::SemanticLegend *CameraSettings::semanticLegend() const { return semantic_legend_.get(); }

const CameraCalibration &CameraSettings::calibration() const { return calibration_; }

const CameraSettings::PhotometricCalibration &CameraSettings::photometricCalibration() const {
  return photometric_calibration_;
}
const CameraSettings::Vignetting &CameraSettings::vignetting() const { return vignetting_; }
const calibration::CameraMask &CameraSettings::cameraMask() const { return camera_mask_; }

}  // namespace dsopp::sensors::calibration
