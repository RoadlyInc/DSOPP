#include "features/camera/eigen_tracking_features_extractor.hpp"

#include <numbers>
#include <numeric>
#include <random>

#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#define DEBUG_TRACKING_FEATURES_EXTRACTOR COMPILE_HIDDEN_CODE

namespace dsopp {
namespace features {
namespace {
/** maximum image pyramid depth */
const int kMaxPyramidDepth = 5;
/**
 * \brief RandomDirection generates an array of the random directions
 *
 * RandomDirection generates an array of the random directions from -PI/2 to PI/2
 *
 * @tparam number of directions
 */
template <unsigned long N>
struct RandomDirections {
  constexpr RandomDirections() : values() {
    for (unsigned long i = 0; i != N; i++) {
      Precision alpha =
          -std::numbers::pi_v<Precision> / 2 + (std::numbers::pi_v<Precision> / N) * static_cast<Precision>(i);
      values[i].first =  // cos
          1 - alpha * alpha / (1 * 2) + alpha * alpha * alpha * alpha / (1 * 2 * 3 * 4) -
          alpha * alpha * alpha * alpha * alpha * alpha / (1 * 2 * 3 * 4 * 5 * 6) +
          alpha * alpha * alpha * alpha * alpha * alpha * alpha * alpha / (1 * 2 * 3 * 4 * 5 * 6 * 7 * 8);
      values[i].second =  // sin
          alpha - alpha * alpha * alpha / (1 * 2 * 3) + alpha * alpha * alpha * alpha * alpha / (1 * 2 * 3 * 4 * 5) -
          alpha * alpha * alpha * alpha * alpha * alpha * alpha / (1 * 2 * 3 * 4 * 5 * 6 * 7) +
          alpha * alpha * alpha * alpha * alpha * alpha * alpha * alpha * alpha / (1 * 2 * 3 * 4 * 5 * 6 * 7 * 8 * 9);
    }
  }
  std::array<std::pair<Precision, Precision>, N> values;
};
constexpr auto kRandomDirections = RandomDirections<16>();
Precision pow(Precision x, size_t y) {
  Precision r = 1;
  for (size_t i = 0; i < y; i++) {
    r *= x;
  }
  return r;
}
/**
 * function for finding the median average of the given vector
 *
 * @param vector given vector
 * @return median average
 */
int computeMedian(std::vector<int> &vector) {
  int threshold = static_cast<int>(std::round(std::accumulate(vector.begin(), vector.end(), 0) * 0.5));
  for (size_t i = 0; i < vector.size(); i++) {
    threshold -= vector[i];
    if (threshold < 0) return static_cast<int>(i);
  }
  return 0;
}
/**
 * function for filtering the given matrix by the median filter
 *
 * @param raw given matrix for filtering
 * @param[out] filtered filtered matrix
 */
void medianFilter(std::vector<std::vector<Precision>> &raw, std::vector<std::vector<Precision>> &filtered) {
  size_t width = filtered.size();
  size_t height = filtered[0].size();
  for (size_t y = 0; y < height; y++) {
    for (size_t x = 0; x < width; x++) {
      Precision sum = 0, num_of_points = 0;
      for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
          int xi = static_cast<int>(x) + i;
          int yj = static_cast<int>(y) + j;
          if ((xi < 0) or (xi > static_cast<int>(width - 1)) or (yj < 0) or (yj > static_cast<int>(height - 1))) {
            continue;
          }
          num_of_points++;
          sum += raw[static_cast<size_t>(xi)][static_cast<size_t>(yj)];
        }
      }
      filtered[x][y] = (sum / num_of_points) * (sum / num_of_points);
    }
  }
}
/**
 * function for fill the map by the average filtered values of the gradient of the intensity
 *
 * @param image map that stores intensity values of the initial image
 * @param[out] gradient_threshold_map map that stores average filtered values of the gradient of the intensity
 * @param mask image detection mask
 */
void fillGradientThresholdMap(const PixelMap<1> &image, std::vector<std::vector<Precision>> &gradient_threshold_map,
                              const sensors::calibration::CameraMask &mask) {
  size_t kMaxGradientLength = 50;
  Precision kMinGradient = 7;

  size_t width = static_cast<size_t>(image.width());
  size_t height = static_cast<size_t>(image.height());
  size_t threshold_map_width = gradient_threshold_map.size();
  size_t threshold_map_height = gradient_threshold_map[0].size();
  std::vector<std::vector<Precision>> gradient_threshold_map_raw(threshold_map_width,
                                                                 std::vector<Precision>(threshold_map_height, 0));
  size_t window_width = width / threshold_map_width;
  size_t window_height = height / threshold_map_height;

  for (size_t y = 0; y < threshold_map_height; y++) {
    for (size_t x = 0; x < threshold_map_width; x++) {
      std::vector<int> gradient_length_counter(kMaxGradientLength, 0);
      for (int yj = static_cast<int>(std::max(window_height * y, 1ul));
           yj < static_cast<int>(std::min(window_height * (y + 1), height - 2)); yj++) {
        for (int xi = static_cast<int>(std::max(window_width * x, 1ul));
             xi < static_cast<int>(std::min(window_width * (x + 1), width - 2)); xi++) {
          if (!mask.valid(xi, yj)) continue;

          const Precision dx = image(xi, yj).jacobian()(0, 0);
          const Precision dy = image(xi, yj).jacobian()(0, 1);
          size_t gradient_length = static_cast<size_t>(
              std::min(std::sqrt(dx * dx + dy * dy), static_cast<Precision>(kMaxGradientLength - 1)));
          gradient_length_counter[gradient_length]++;
        }
      }
      gradient_threshold_map_raw[x][y] = static_cast<Precision>(computeMedian(gradient_length_counter)) + kMinGradient;
    }
  }

  medianFilter(gradient_threshold_map_raw, gradient_threshold_map);
}
/**
 * function for calculating the new potential to get the count of points closing to the desired number
 *
 * @param point_ratio ratio of the desired number of points to the founded
 * @param current_potential current potential which needs to be modified
 */
int calculatePotential(Precision point_ratio, int current_potential) {
  int ideal_potential = static_cast<int>(std::sqrt(1.0 / point_ratio) * (current_potential + 1) - 1);
  if (ideal_potential < 1) {
    ideal_potential = 1;
  }
  return ideal_potential;
}
/**
 * function for directly reducing the number of features
 *
 * @param point_ratio ratio of the desired number of points to the founded
 * @param[out] feature_coordinates vector to fill in features coordinates
 */
size_t reduceTheNumberOfPoints(Precision point_ratio, std::vector<Eigen::Vector2<Precision>> &feature_coordinates,
                               const std::vector<uint8_t> &random_pattern, long width) {
  int threshold = static_cast<int>(255. * point_ratio);
  auto to_erase =
      std::remove_if(feature_coordinates.begin(), feature_coordinates.end(), [&](const Eigen::Vector2<Precision> &p) {
        return random_pattern[static_cast<size_t>(p.x() + p.y() * static_cast<Precision>(width))] > threshold;
      });

  feature_coordinates.erase(to_erase, feature_coordinates.end());
  return feature_coordinates.size();
}
/**
 * function for finding features in image pyramid with given potential, which defines the features density.
 *
 * @param pixel_frame pyramid of the images
 * @param gradient_threshold_map map that stores average filtered values of the gradient of the intensity
 * @param x, y coordinates of the candidate to the feature
 * @param potential current potential, which defines the number of founded features.
 * @param random_direction vector of the random directions to choose feature from candidates
 * @param quality_level parameter that defines the quality of the feature point. If it is high, only
 * points with a high gradient can become a feature.
 * @param[out] candidate_coords, candidate_weight parameters of the candidate to the feature on each level
 * @param kBorderSize border size
 */
void findBestCandidate(const PixelDataFrame &pixel_frame, std::vector<std::vector<Precision>> &gradient_threshold_map,
                       int x, int y, std::vector<std::pair<Precision, Precision>> &random_direction,
                       Precision quality_level, std::vector<Eigen::Vector<Precision, 2>> &candidate_coords,
                       std::vector<Precision> &candidate_weight, int kBorderSize) {
  Precision kGradDownweightPerLevel = 0.75;
  size_t depth = pixel_frame.size();
  long width = pixel_frame.getLevel(0).width();
  long height = pixel_frame.getLevel(0).height();
  size_t threshold_map_width = gradient_threshold_map.size();
  size_t threshold_map_height = gradient_threshold_map[0].size();

  if (x < kBorderSize or x >= width - 1 - kBorderSize or y < kBorderSize or y > height - 1 - kBorderSize) {
    return;
  }

  size_t xi = static_cast<size_t>(x) >> kMaxPyramidDepth;
  size_t yj = static_cast<size_t>(y) >> kMaxPyramidDepth;
  if (xi >= threshold_map_width or yj >= threshold_map_height) {
    return;
  }

  Precision threshold = gradient_threshold_map[xi][yj];
  for (size_t level = 0; level < depth; level++) {
    auto &direction = random_direction[level];
    Precision downgrade = 1.0_p / static_cast<Precision>(1 << level);
    threshold = threshold * pow(kGradDownweightPerLevel, level);
    if (candidate_coords[level](0) == -2) {
      continue;
    }
    auto &mapmax = pixel_frame.getLevel(level);
    int x_i = static_cast<int>(static_cast<Precision>(x) * downgrade);
    int y_i = static_cast<int>(static_cast<Precision>(y) * downgrade);
    if (x_i < kBorderSize || x_i >= width - 1 - kBorderSize || y_i < kBorderSize || y_i > height - 1 - kBorderSize) {
      return;
    }

    auto &pixel = mapmax(x_i, y_i);

    const Precision dx = pixel.jacobian()(0, 0);
    const Precision dy = pixel.jacobian()(0, 1);

    Precision grad_length_sqr = dx * dx + dy * dy;
    Precision grad_projection =
        std::abs(Eigen::Vector<Precision, 2>(direction.first, direction.second).dot(pixel.jacobian()));
    if (grad_length_sqr > threshold * quality_level and grad_projection > candidate_weight[level]) {
      candidate_weight[level] = grad_length_sqr;
      candidate_coords[level] = Eigen::Vector<Precision, 2>(x, y);
      for (size_t next = level + 1; next < depth; next++) {
        candidate_coords[next] = Eigen::Vector<Precision, 2>(-2, -2);
      }
    }
  }
}
/**
 * function for finding features in image pyramid with given potential, which defines the features density.
 *
 * @param pixel_frame pyramid of the images
 * @param gradient_threshold_map map that stores average filtered values of the gradient of the intensity
 * @param level current level of pyramid
 * @param x_start, y_start lower left corner of the given part of the image
 * @param potential current potential, which defines the number of founded features.
 * @param random_direction vector of the random directions to choose feature from candidates
 * @param candidate_coords, candidate_weight parameters of the candidate to the feature on each level
 * @param quality_level parameter that defines the quality of the feature point. If it is high, only
 * points with a high gradient can become a feature.
 * @param[out] feature_coordinates vector to fill in features coordinates
 * @param[out] num_of_features vector, which counts features on each level
 * @param kBorderSize border size
 * @param mask image roi mask
 */
void findFeaturesInWindow(const PixelDataFrame &pixel_frame,
                          std::vector<std::vector<Precision>> &gradient_threshold_map, size_t level, int x_start,
                          int y_start, int potential, std::vector<Eigen::Vector<Precision, 2>> &candidate_coords,
                          std::vector<Precision> &candidate_weight,
                          std::vector<std::pair<Precision, Precision>> &random_direction, Precision quality_level,
                          std::vector<Eigen::Vector2<Precision>> &feature_coordinates,
                          std::vector<int> &num_of_features, int kBorderSize,
                          const sensors::calibration::CameraMask &mask, const std::vector<uint8_t> &random_pattern) {
  size_t depth = pixel_frame.size();
  candidate_coords[level] = Eigen::Vector<Precision, 2>(-1, -1);
  candidate_weight[level] = 0;
  int window = (1 << level) * potential;
  int step = window / 2;
  if (level == 0) {
    step = 1;
  }
  for (int y = y_start; y < y_start + window; y += step) {
    for (int x = x_start; x < x_start + window; x += step) {
      if (!mask.valid(x, y)) continue;

      random_direction[level] = kRandomDirections.values[random_pattern[feature_coordinates.size()] & 0xF];

      if (level > 0) {
        findFeaturesInWindow(pixel_frame, gradient_threshold_map, level - 1, x, y, potential, candidate_coords,
                             candidate_weight, random_direction, quality_level, feature_coordinates, num_of_features,
                             kBorderSize, mask, random_pattern);
      } else {
        findBestCandidate(pixel_frame, gradient_threshold_map, x, y, random_direction, quality_level, candidate_coords,
                          candidate_weight, kBorderSize);
      }
    }
  }
  if (candidate_coords[level](0) > 0) {
    auto &candidate = candidate_coords[level];
    feature_coordinates.emplace_back(candidate);
    num_of_features[level]++;
    random_direction[level] =
        kRandomDirections
            .values[random_pattern[static_cast<size_t>(
                        candidate.x() + candidate.y() * static_cast<Precision>(pixel_frame.getLevel(level).width()))] &
                    0xF];
    if (level + 1 < depth) {
      candidate_weight[level + 1] = 1e10;
    }
  }
}
/**
 * function for finding features in image pyramid with given potential, which defines the features density.
 *
 * @param pixel_frame pyramid of the images
 * @param gradient_threshold_map map that stores average filtered values of the gradient of the intensity
 * @param current_potential current potential, which defines the number of founded features.
 * @param quality_level parameter that defines the quality of the feature point. If it is high, only
 * points with a high gradient can become a feature.
 * @param[out] feature_coordinates vector to fill in features coordinates
 * @param[out] num_of_features vector, which counts features on each level
 * @param kBorderSize border size
 * @param mask image roi mask
 */
int findFeatures(const PixelDataFrame &pixel_frame, std::vector<std::vector<Precision>> &gradient_threshold_map,
                 Precision quality_level, int potential, std::vector<Eigen::Vector2<Precision>> &feature_coordinates,
                 std::vector<int> &num_of_features, int kBorderSize, const sensors::calibration::CameraMask &mask,
                 const std::vector<uint8_t> &random_pattern) {
  size_t depth = pixel_frame.size();
  long width = pixel_frame.getLevel(0).width();
  long height = pixel_frame.getLevel(0).height();
  int step = (1 << (depth - 1)) * potential;

  std::vector<Eigen::Vector<Precision, 2>> candidate_coords(depth);
  std::vector<Precision> candidate_weight(depth);

  std::vector<std::pair<Precision, Precision>> random_direction(depth);
  for (size_t i = 0; i < depth; i++) {
    random_direction[i] = kRandomDirections.values[random_pattern[i] & 0xF];
  }
  //
  for (int y = 0; y < height; y += step) {
    for (int x = 0; x < width; x += step) {
      findFeaturesInWindow(pixel_frame, gradient_threshold_map, depth - 1, x, y, potential, candidate_coords,
                           candidate_weight, random_direction, quality_level, feature_coordinates, num_of_features,
                           kBorderSize, mask, random_pattern);
    }
  }
  return std::accumulate(num_of_features.begin(), num_of_features.end(), 0);
}
/**
 * function for fill the map of the pixels type. Type -1 means that pixel does not feature.
 * Other values mean the level on which feature was found. Function called recursively
 * with other value of the parameter potential, which defines the number of founded features.
 *
 * @param pixel_frame pyramid of the images
 * @param gradient_threshold_map map that stores average filtered values of the gradient of the intensity
 * @param desired_points desired number of the feature points
 * @param current_potential current potential, which defines the number of founded features.
 * @param recursions_left parameter to limit the number of recursions (sometimes desired number of the
 * feature point can't be reached)
 * @param quality_level parameter that defines the quality of the feature point. If it is high, only
 * points with a high gradient can become a feature.
 * @param[out] feature_coordinates vector to fill in features coordinates
 * @param kBorderSize border size
 * @param mask image roi mask
 */
void fillFeatureCoordinates(const PixelDataFrame &pixel_frame,
                            std::vector<std::vector<Precision>> &gradient_threshold_map, Precision desired_points,
                            int &current_potential, int recursions_left, Precision quality_level,
                            std::vector<Eigen::Vector2<Precision>> &feature_coordinates, int kBorderSize,
                            const sensors::calibration::CameraMask &mask, const std::vector<uint8_t> &random_pattern) {
  size_t depth = pixel_frame.size();
  std::vector<int> num_of_features(depth);
  feature_coordinates.clear();

  int found_points = findFeatures(pixel_frame, gradient_threshold_map, quality_level, current_potential,
                                  feature_coordinates, num_of_features, kBorderSize, mask, random_pattern);
  Precision point_ratio = desired_points / static_cast<Precision>(found_points);
  int ideal_potential = calculatePotential(point_ratio, static_cast<int>(current_potential));

  if (recursions_left > 0 and point_ratio > 1.25 and current_potential > 1) {
    // re-sample to get more points! potential needs to be smaller
    if (ideal_potential >= current_potential) {
      ideal_potential = current_potential - 1;
    }
    current_potential = ideal_potential;
    return fillFeatureCoordinates(pixel_frame, gradient_threshold_map, desired_points, current_potential,
                                  recursions_left - 1, quality_level, feature_coordinates, kBorderSize, mask,
                                  random_pattern);
  } else if (recursions_left > 0 and point_ratio < 0.25) {
    // re-sample to get less points! potential needs to be bigger
    if (ideal_potential <= current_potential) {
      ideal_potential = current_potential + 1;
    }
    current_potential = ideal_potential;
    return fillFeatureCoordinates(pixel_frame, gradient_threshold_map, desired_points, current_potential,
                                  recursions_left - 1, quality_level, feature_coordinates, kBorderSize, mask,
                                  random_pattern);
  }
  // all recursions have done, need to reduce the number of points directly
  if (point_ratio < 0.95) {
    reduceTheNumberOfPoints(point_ratio, feature_coordinates, random_pattern, pixel_frame.getLevel(0).width());
  }
}

void trackingFeaturesBuilder(std::vector<Eigen::Vector<Precision, 2>> &feature_coordinates,
                             std::vector<TrackingFeature> &tracking_features) {
  tracking_features.reserve(feature_coordinates.size());

  for (size_t i = 0; i < feature_coordinates.size(); i++) {
    auto &coordinates = feature_coordinates.at(i);
    tracking_features.emplace_back(coordinates);
  }
}
/**
 * function to detect features for tracking on the grayscale image
 *
 * @param pixel_frame pyramid of the images
 * @param[out] feature_coordinates vector to fill in features coordinates
 * @param point_density desired number of the feature points
 * @param quality_level parameter that defines the quality of the feature point. If it is high, only
 * points with a high gradient can become a feature.
 * @param initial_potential initial potential, which defines the number of founded features.
 * @param mask image detection mask
 * @param kBorderSize border size
 */
void detectFeaturesToTrack(const PixelDataFrame &pixel_frame,
                           std::vector<Eigen::Vector<Precision, 2>> &feature_coordinates, const Precision point_density,
                           const Precision quality_level, int &current_potential,
                           const sensors::calibration::CameraMask &mask, int kBorderSize,
                           const std::vector<uint8_t> &random_pattern) {
  auto &image = pixel_frame.getLevel(0);
  auto width = static_cast<size_t>(image.width());
  auto height = static_cast<size_t>(image.height());
  // Gradient threshold map
  std::vector<std::vector<Precision>> gradient_threshold_map(
      width / (1 << kMaxPyramidDepth), std::vector<Precision>(height / (1 << kMaxPyramidDepth), 0));
  fillGradientThresholdMap(image, gradient_threshold_map, mask);
  fillFeatureCoordinates(pixel_frame, gradient_threshold_map, point_density, current_potential, 1, quality_level,
                         feature_coordinates, kBorderSize, mask, random_pattern);
}
}  // namespace

EigenTrackingFeaturesExtractor::EigenTrackingFeaturesExtractor(const Precision point_density_for_detector)
    : TrackingFeaturesExtractor(point_density_for_detector) {}

std::unique_ptr<TrackingFeaturesFrame> EigenTrackingFeaturesExtractor::extract(
    cv::Mat image, const sensors::calibration::CameraMask &mask) {
  const Precision kQualityLevelForDetector = 1;
  // Note: here we construct new raw pyramids (according to DSO paper) instead of reusing photometrically corrected
  // pyramids
  features::PixelDataFrame::PhotometricCalibration photometric_calibration;
  std::iota(photometric_calibration.begin(), photometric_calibration.end(), 0);
  features::PixelDataFrame pixel_frame(image, photometric_calibration, features::PixelDataFrame::Vignetting(),
                                       features::PixelDataFrame::kMaxPyramidDepth);

  std::vector<uint8_t> random_pattern(
      static_cast<size_t>(pixel_frame.getLevel(0).width() * pixel_frame.getLevel(0).height()));
  std::srand(3141592);  // want to be deterministic.
  for (uint8_t &i : random_pattern) i = static_cast<uint8_t>(rand());

  std::vector<Eigen::Vector2<Precision>> feature_coordinates;
  feature_coordinates.reserve(static_cast<size_t>(point_density_for_detector_));
  detectFeaturesToTrack(pixel_frame, feature_coordinates, point_density_for_detector_, kQualityLevelForDetector,
                        current_potential_, mask.getEroded(kBorderSize).getEroded(kGradientBorder), kBorderSize,
                        random_pattern);
#if DEBUG_TRACKING_FEATURES_EXTRACTOR
  auto &pixel_map = pixel_frame.getLevel(0);
  cv::Mat debug(static_cast<int>(pixel_map.height()), static_cast<int>(pixel_map.width()), CV_8UC1, cv::Scalar(0));
  for (int x = 0; x < pixel_map.width(); x++) {
    for (int y = 0; y < pixel_map.height(); y++) {
      debug.at<uint8_t>(cv::Point(x, y)) = static_cast<uint8_t>(pixel_map(x, y).intensity());
    }
  }
  for (auto &c : feature_coordinates) {
    cv::circle(debug, cv::Point(static_cast<int>(c[0]), static_cast<int>(c[1])), 2, cv::Scalar(255), 1);
  }
  cv::imshow("tracking features", debug);
  cv::waitKey(0);
#endif
  std::vector<TrackingFeature> tracking_features;
  trackingFeaturesBuilder(feature_coordinates, tracking_features);
  return std::make_unique<TrackingFeaturesFrame>(std::move(tracking_features));
}

}  // namespace features
}  // namespace dsopp
