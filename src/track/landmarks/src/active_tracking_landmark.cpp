#include "track/landmarks/active_tracking_landmark.hpp"

#include <algorithm>

#include "track/landmarks/immature_tracking_landmark.hpp"

namespace dsopp {
namespace track {
namespace landmarks {

ActiveTrackingLandmark::ActiveTrackingLandmark(const Eigen::Vector3<Precision>& direction,
                                               const Eigen::Vector2<Precision>& projection,
                                               const Eigen::Vector<Precision, Pattern::kSize>& patch, Precision idepth)
    : TrackingLandmarkWithPatch(direction, projection, patch),
      idepth_(idepth),
      idepth_variance_(std::numeric_limits<Precision>::max()),
      relative_baseline_(0),
      is_marginalized_(false),
      number_of_successful_optimizations_(0),
      number_of_inlier_residuals_in_the_last_optimization_(0) {}

Precision ActiveTrackingLandmark::idepth() const { return idepth_; }

Precision ActiveTrackingLandmark::relativeBaseline() const { return relative_baseline_; }

Precision ActiveTrackingLandmark::idepthVariance() const { return idepth_variance_; }

void ActiveTrackingLandmark::setIdepth(Precision idepth) {
  assert(not is_marginalized_);
  idepth_ = idepth;
}

void ActiveTrackingLandmark::setRelativeBaseline(Precision relative_baseline) {
  assert(not is_marginalized_);
  relative_baseline_ = relative_baseline;
}

void ActiveTrackingLandmark::setIdepthVariance(Precision idepth_variance) { idepth_variance_ = idepth_variance; }

size_t ActiveTrackingLandmark::numberOfSuccessfulOptimizations() { return number_of_successful_optimizations_; }

void ActiveTrackingLandmark::setNumberOfInlierResidualsInTheLastOptimization(
    size_t number_of_inlier_residuals_in_the_last_optimization) {
  const size_t kMinInlierResidualsForSuccessfulOptimization = 3;
  number_of_inlier_residuals_in_the_last_optimization_ = number_of_inlier_residuals_in_the_last_optimization;
  if (number_of_inlier_residuals_in_the_last_optimization > kMinInlierResidualsForSuccessfulOptimization) {
    ++number_of_successful_optimizations_;
  }
}

size_t ActiveTrackingLandmark::numberOfInlierResidualsInTheLastOptimization() {
  return number_of_inlier_residuals_in_the_last_optimization_;
}

void ActiveTrackingLandmark::marginalize() {
  const Precision kMaxRelativeIdepthVariance = 1e-3_p;

  if ((this->idepthVariance() / idepth_) > kMaxRelativeIdepthVariance || idepth_ < 0) {
    this->markOutlier();
  }
  is_marginalized_ = true;
  patch_.release();
}

bool ActiveTrackingLandmark::isMarginalized() const { return is_marginalized_; }

void ActiveTrackingLandmark::addSemanticTypeObservation(size_t semantic_type_id) {
  semantic_type_observations_[semantic_type_id]++;
}

size_t ActiveTrackingLandmark::semanticTypeId(const semantics::SemanticLegend* legend) const {
  long maximum_observation_element =
      std::max_element(semantic_type_observations_.begin(), semantic_type_observations_.end()) -
      semantic_type_observations_.begin();
  if (!legend) {
    return static_cast<size_t>(maximum_observation_element);
  }
  size_t max_weight_element = 0;
  size_t max_weight = 0;
  for (size_t i = 0; i < semantic_type_observations_.size(); ++i) {
    size_t weight = semantic_type_observations_[i] * legend->weight(i);
    if (weight > max_weight) {
      max_weight_element = i;
      max_weight = weight;
    }
  }
  return (max_weight == 0) ? static_cast<size_t>(maximum_observation_element) : max_weight_element;
}

}  // namespace landmarks
}  // namespace track
}  // namespace dsopp
