#ifndef DSOPP_SENSORS_CAMERA_CALIBRATION_CAMERA_SETTINGS_HPP_
#define DSOPP_SENSORS_CAMERA_CALIBRATION_CAMERA_SETTINGS_HPP_

#include <memory>
#include <opencv2/opencv.hpp>

#include "camera_settings.pb.h"
#include "common/settings.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp::sensors::calibration {

/**
 * \brief Storing all camera calibration parameters
 *
 * @tparam Calibration camera calibration type
 */
class CameraSettings {
 public:
  /** Alias for photometric calibration.*/
  using PhotometricCalibration = std::array<Precision, 256>;
  /** Alias for vignetting.*/
  using Vignetting = cv::Mat;
  /**
   * @param calibration camera calibration
   * @param photometric_calibration photometric calibration
   * @param vignetting vignetting
   * @param camera_mask camera mask
   * @param semantic_legend semantics legend
   */
  CameraSettings(CameraCalibration &&calibration, PhotometricCalibration &&photometric_calibration,
                 Vignetting &&vignetting, calibration::CameraMask &&camera_mask,
                 std::unique_ptr<semantics::SemanticLegend> &&semantic_legend = nullptr);

  /**
   * @param camera_settings protobuf message
   */
  CameraSettings(const proto::CameraSettings &camera_settings);
  /**
   * move constructor
   */
  CameraSettings(CameraSettings &&);
  /**
   * Create proto message with calibration
   * @return proto message
   */
  proto::CameraSettings proto() const;
  /**
   * @return deep copy of the object
   */
  CameraSettings clone() const;
  /**
   * @param parameters parameters
   */
  void setCameraIntrinsics(const Eigen::VectorX<Precision> &parameters);
  /**
   * @return camera calibration
   */
  const CameraCalibration &calibration() const;
  /**
   * @return photometric_calibration
   */
  const PhotometricCalibration &photometricCalibration() const;
  /**
   * @return vignetting
   */
  const Vignetting &vignetting() const;
  /**
   * @return camera_mask
   */
  const calibration::CameraMask &cameraMask() const;
  /**
   * @return semantics legend if exists and false otherwise
   */
  const semantics::SemanticLegend *semanticLegend() const;

 protected:
  /**
   * creates deep copy of the object
   */
  CameraSettings(const CameraSettings &);

 private:
  /** Camera calibration */
  CameraCalibration calibration_;
  /** Photometric calibration.*/
  PhotometricCalibration photometric_calibration_;
  /** Vignetting.*/
  Vignetting vignetting_;
  /** Camera mask */
  calibration::CameraMask camera_mask_;
  /** semantic legend */
  std::unique_ptr<semantics::SemanticLegend> semantic_legend_;
};

}  // namespace dsopp::sensors::calibration

#endif  // DSOPP_SENSORS_CAMERA_CALIBRATION_CAMERA_SETTINGS_HPP_
