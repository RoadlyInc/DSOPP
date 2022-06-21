#include "features/camera/sobel_tracking_features_extractor.hpp"

#include <optional>
#include <random>

#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp {
namespace features {
namespace {

/**
 * function for calculating the new minimum of gradient norm threshold to get the count of points closing to the desired
 * number Note: we approximate the distribution of the gradient norm by an exponential distribution
 *
 * @param num_pixels number of pixels into image
 * @param desired_points number of desired points
 * @param found_points number of found points
 * @param current_threshold current threshold which needs to be modified
 * @return new threshold
 */
int calculateThreshold(int num_pixels, int desired_points, int found_points, int current_threshold) {
  return static_cast<int>(current_threshold * std::log(num_pixels / desired_points) /
                          std::log(num_pixels / found_points));
}

/**
 * calculate q-quantile of gradient norms
 *
 * @param grad_norm matrix of gradient norm squared for all pixels
 * @param q quantile level
 * @return q-quantile of gradient norms
 */
int16_t quantile(cv::Mat &grad_norm, Precision q) {
  std::vector<int16_t> grad_norms(grad_norm.begin<int16_t>(), grad_norm.end<int16_t>());
  std::nth_element(grad_norms.begin(),
                   grad_norms.begin() + static_cast<long>(static_cast<Precision>(grad_norms.size()) * q),
                   grad_norms.end());

  return static_cast<int16_t>(grad_norms[static_cast<size_t>(static_cast<Precision>(grad_norms.size()) * q)]);
}

/**
 * find point with high gradient in window_size X window_size block
 *
 * @param grad_norm matrix of gradient norm for all pixels
 * @param mask mask for valid points
 * @param window_size size of window in which point is searched
 * @param window_start_x x coord of start point in common matrix
 * @param window_start_y y coord of start point in common matrix
 * @param grad_norm_threshold threshold for gradient norm
 * @return first point with enough high gradient if it exist
 */
std::optional<Eigen::Matrix<Precision, 2, 1>> findPointInWindow(cv::Mat &grad_norm,
                                                                const sensors::calibration::CameraMask &mask,
                                                                int window_size, int window_start_x, int window_start_y,
                                                                int grad_norm_threshold) {
  for (int y = window_start_y; y < window_start_y + window_size; ++y) {
    for (int x = window_start_x; x < window_start_x + window_size; ++x) {
      if (grad_norm.at<int16_t>(y, x) > grad_norm_threshold && mask.valid(x, y)) {
        return Eigen::Matrix<Precision, 2, 1>(x, y);
      }
    }
  }
  return std::nullopt;
}
}  // namespace

SobelTrackingFeaturesExtractor::SobelTrackingFeaturesExtractor(const Precision point_density_for_detector,
                                                               const Precision quantile_level)
    : TrackingFeaturesExtractor(point_density_for_detector), quantile_level_(quantile_level) {}

std::unique_ptr<TrackingFeaturesFrame> SobelTrackingFeaturesExtractor::extract(
    cv::Mat pixel_frame, const sensors::calibration::CameraMask &camera_mask) {
  std::vector<TrackingFeature> tracking_features;
  tracking_features.reserve(10000);

  cv::Mat canny_x, canny_y, canny_x_squared, canny_y_squared;
  cv::Sobel(pixel_frame, canny_x, CV_16S, 1, 0);
  canny_x_squared = canny_x.mul(canny_x);

  cv::Sobel(pixel_frame, canny_y, CV_16S, 0, 1);
  canny_y_squared = canny_y.mul(canny_y);

  cv::Mat canny_x_abs = cv::abs(canny_x);
  cv::Mat canny_y_abs = cv::abs(canny_y);
  cv::Mat grad_norm = canny_x_abs + canny_y_abs;  // L1 norm is used due it's more simpler and to avoid overflow

  auto rng = std::default_random_engine{};
  auto mask = camera_mask.getEroded(kBorderSize).getEroded(kGradientBorder);

  if (!initialized_) {
    initialized_ = true;
    grad_norm_threshold_ = quantile(grad_norm, quantile_level_);
    Precision potential = std::sqrt(static_cast<Precision>(pixel_frame.rows * pixel_frame.cols) *
                                    (1_p - quantile_level_) / point_density_for_detector_);
    Precision kMinPotential = 1_p;
    if (potential < kMinPotential) {
      point_density_for_detector_ *= potential * potential / (kMinPotential * kMinPotential);
      potential = kMinPotential;
    }
    current_potential_ = static_cast<int>(potential);
  }

  int window_size = current_potential_;

  for (int y = 0; y + window_size < grad_norm.rows; y += window_size) {
    for (int x = 0; x + window_size < grad_norm.cols; x += window_size) {
      auto point = findPointInWindow(grad_norm, mask, window_size, x, y, grad_norm_threshold_);
      if (point) {
        tracking_features.emplace_back(*point);
      }
    }
  }

  auto found_points = tracking_features.size();

  grad_norm_threshold_ =
      calculateThreshold(pixel_frame.rows * pixel_frame.cols, static_cast<int>(point_density_for_detector_),
                         static_cast<int>(found_points), grad_norm_threshold_);

  std::shuffle(tracking_features.begin(), tracking_features.end(), rng);
  if (static_cast<Precision>(tracking_features.size()) > point_density_for_detector_) {
    tracking_features.erase(tracking_features.begin() + static_cast<long>(point_density_for_detector_),
                            tracking_features.end());
  }

  return std::make_unique<TrackingFeaturesFrame>(std::move(tracking_features));
}
}  // namespace features
}  // namespace dsopp
