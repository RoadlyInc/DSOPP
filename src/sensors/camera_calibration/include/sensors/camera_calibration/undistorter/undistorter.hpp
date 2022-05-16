
#ifndef DSOPP_SRC_SENSORS_CAMERA_CALIBRATION_UNDISTORTER_UNDISTORTER_HPP_
#define DSOPP_SRC_SENSORS_CAMERA_CALIBRATION_UNDISTORTER_UNDISTORTER_HPP_

#include <limits>
#include <memory>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "common/settings.hpp"
#include "energy/camera_model/distroted_camera.hpp"
#include "energy/camera_model/pinhole/ios_camera_model.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"

namespace dsopp::sensors::calibration {
/**
 *
 * \brief generates remap from any camera model to pinhole model.
 * Target Pinhole Model parameters are evaluated based on FOV with center in image center.
 *
 * Horizontal/Vertical fov (focal) -- based on minimum distance between projected image borders
 */
class Undistorter {
  /** alias for PinholeCamera */
  using PinholeCamera = energy::model::PinholeCamera<Precision>;

  /** Private constructor
   *
   * @param remapX oX lookup table
   * @param remapY oY lookup table
   * @param input_width input image width
   * @param input_height input image height
   * */
  Undistorter(const cv::Mat &remapX, const cv::Mat &remapY, const size_t input_width, const size_t input_height)
      : remapX_(remapX.clone()), remapY_(remapY.clone()), input_width_(input_width), input_height_(input_height) {}

  /**
   * Check if point is inside image size
   *
   * @param point point
   * @param image_size size of image
   * @return bool true if inside, false otherwise
   */
  static bool insideImage(const Eigen::Vector2<Precision> &point, const Eigen::Vector2<Precision> image_size) {
    if (point(0) < 0.0 || point(1) < 0.0) return false;
    if (point(0) > image_size(0) - 1 || point(1) > image_size(1) - 1) return false;

    return true;
  }

 public:
  ~Undistorter() = default;

  /**
   * Method which creates Identity undistorter
   *
   * @param image_size image size
   * @return unique_ptr ot undistorter
   */
  static Undistorter Identity(const Eigen::Vector2<Precision> &image_size);

  /**
   * Method which generates remaps
   *
   * @param from_model initial camera model which to rectify
   * @return estimated PinholeCameraCalibration
   */
  template <dsopp::energy::model::DistortedCamera CameraModel>
  static std::pair<PinholeCamera, Undistorter> constructRemaps(const CameraModel &from_model) {
    Precision fx = from_model.focalX();
    Precision fy = from_model.focalY();
    const auto &input_image_size = from_model.image_size();
    Eigen::Vector2<Precision> focals(fx, fy);

    Eigen::Vector2<Precision> image_size = input_image_size;

    Eigen::Vector2<Precision> center = image_size / 2;

    auto initial_pinhole = PinholeCamera(image_size, focals, center);

    cv::Mat remapX(static_cast<int>(image_size(1)), static_cast<int>(image_size(0)), CV_32F);
    cv::Mat remapY(static_cast<int>(image_size(1)), static_cast<int>(image_size(0)), CV_32F);

    estimateRemaps(from_model, initial_pinhole, remapX, remapY);

    Undistorter undistorter(remapX, remapY, static_cast<size_t>(input_image_size.x()),
                            static_cast<size_t>(input_image_size.y()));
    return {initial_pinhole, undistorter};
  }

  /**
   * Method to rectify image from sensor
   *
   * @param img input image
   * @param interpolationType openCV interpolation type
   * @return rectified image
   */
  cv::Mat undistort(const cv::Mat &img, int interpolationType = cv::INTER_LINEAR) const;

  /**
   * get input image width
   * @return input image width
   */
  size_t input_width() const;

  /**
   * get input image height
   * @return input image height
   */
  size_t input_height() const;

 private:
  /**
   * Method to estimate remap to pinhole
   *
   * @param from_model initial camera model
   * @param to_pinhole pinhole model onto which to project
   * @param remapX lookup table for oX
   * @param remapY lookup table for oY
   * @return pair of X and Y lookup tables for cv::remap
   */
  template <dsopp::energy::model::DistortedCamera CameraModel>
  static void estimateRemaps(const CameraModel &from_model, const PinholeCamera &to_pinhole, cv::Mat &remapX,
                             cv::Mat &remapY) {
    for (int y = 0; y < remapX.rows; ++y)
      for (int x = 0; x < remapX.cols; ++x) {
        Eigen::Vector2<Precision> point(x, y);
        Eigen::Vector3<Precision> ray;

        bool success = to_pinhole.unproject(point, ray);
        success = success and from_model.project(ray, point);

        if (success) {
          remapX.at<float>(y, x) = static_cast<float>(point(0));
          remapY.at<float>(y, x) = static_cast<float>(point(1));
        } else {
          remapX.at<float>(y, x) = -1;
          remapY.at<float>(y, x) = -1;
        }
      }
  }

  /** Lookup table for remapping x */
  cv::Mat remapX_;
  /** Lookup table for remapping y */
  cv::Mat remapY_;
  /** input iamge width */
  const size_t input_width_;
  /** input iamge height */
  const size_t input_height_;
};

}  // namespace dsopp::sensors::calibration

#endif  // DSOPP_SRC_SENSORS_CAMERA_CALIBRATION_UNDISTORTER_UNDISTORTER_HPP_
