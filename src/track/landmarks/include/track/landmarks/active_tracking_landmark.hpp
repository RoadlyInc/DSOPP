#ifndef DSOPP_SRC_TRACK_LANDMARKS_ACTIVE_TRACKING_LANDMARK_HPP_
#define DSOPP_SRC_TRACK_LANDMARKS_ACTIVE_TRACKING_LANDMARK_HPP_

#include "common/pattern/pattern.hpp"
#include "common/settings.hpp"
#include "semantics/semantic_legend.hpp"
#include "track/landmarks/tracking_landmark_with_patch.hpp"

namespace dsopp {
namespace track {
namespace landmarks {
class ImmatureTrackingLandmark;
/**
 * class to store active 3d landmark. Active landmark has an idepth but it will be optimized.
 */
class ActiveTrackingLandmark : public TrackingLandmarkWithPatch {
 public:
  /**
   * create active point.
   * @param direction direction vector
   * @param projection projection point
   * @param patch patch on the corresponding image
   * @param idepth idepth
   */
  ActiveTrackingLandmark(const Eigen::Vector3<Precision>& direction, const Eigen::Vector2<Precision>& projection,
                         const Eigen::Vector<Precision, Pattern::kSize>& patch, Precision idepth);
  /**
   * @return idepth
   */
  Precision idepth() const;
  /**
   * @return relative baseline
   */
  Precision relativeBaseline() const;
  /**
   * @return idepth variance
   */
  Precision idepthVariance() const;
  /**
   * marginalizes landmarks (sets constant)
   */
  void marginalize();
  /**
   * @return true if marginalized
   */
  bool isMarginalized() const;
  /**
   * @param idepth new idepth value
   */
  void setIdepth(Precision idepth);
  /**
   * @param relative_baseline new relative baseline value
   */
  void setRelativeBaseline(Precision relative_baseline);
  /**
   * @param idepth_variance new idepth variance value
   */
  void setIdepthVariance(Precision idepth_variance);
  /**
   * @return number of successful optimizations in history
   */
  size_t numberOfSuccessfulOptimizations();
  /**
   * @param number_of_inlier_residuals_in_the_last_optimization number of inlier residuals in the last optimization
   */
  void setNumberOfInlierResidualsInTheLastOptimization(size_t number_of_inlier_residuals_in_the_last_optimization);
  /**
   * @return number of inlier residuals in the last optimization
   */
  size_t numberOfInlierResidualsInTheLastOptimization();
  /**
   * adds semantic type observation of landmark (ref: ``semantic_legend.hpp``)
   * @param semantic_type_id semantic type
   */
  void addSemanticTypeObservation(size_t semantic_type_id);

  /**
   * get semantic type
   * @param legend semantics legend to get weight of the tags. If legend if nullptr, the type with the largest
   * observations will be returned
   * @return semantic type
   */
  size_t semanticTypeId(const semantics::SemanticLegend* legend = nullptr) const;

 private:
  /** idepth */
  Precision idepth_;
  /** idepth variance */
  Precision idepth_variance_;
  /** relative baseline (ratio between baseline in optimization window and depth) */
  Precision relative_baseline_;
  /** true if marginalized*/
  bool is_marginalized_;
  /** number of successful optimizations in history*/
  size_t number_of_successful_optimizations_;
  /** number of inlier residuals in the last optimization */
  size_t number_of_inlier_residuals_in_the_last_optimization_;
  /** semantic type of landmark */
  std::array<uint8_t, semantics::SemanticLegend::kMaxNumberOfTypes> semantic_type_observations_ = {0};
};
}  // namespace landmarks
}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACK_LANDMARKS_ACTIVE_TRACKING_LANDMARK_HPP_
