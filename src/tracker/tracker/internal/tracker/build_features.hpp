#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "common/settings.hpp"
#include "features/camera/tracking_features_frame.hpp"

namespace dsopp {
namespace tracker {
/**
 * Function to create features with direction from the 2d coordinates and the camera model
 * @tparam Model camera model
 *
 * @param features_frame 2d features
 * @param model camera model to unproject features
 */
template <typename Model>
std::vector<std::pair<Eigen::Vector2<Precision>, Eigen::Vector3<Precision>>> buildFeatures(
    const features::TrackingFeaturesFrame &features_frame, const Model &model) {
  std::vector<std::pair<Eigen::Vector2<Precision>, Eigen::Vector3<Precision>>> features;
  for (const auto &feature : features_frame.features()) {
    const auto &coordinates = feature.coordinates();
    Eigen::Vector3<Precision> direction;
    if (model.unproject(coordinates, direction)) {
      features.emplace_back(coordinates, direction);
    }
  }
  return features;
}
}  // namespace tracker
}  // namespace dsopp
