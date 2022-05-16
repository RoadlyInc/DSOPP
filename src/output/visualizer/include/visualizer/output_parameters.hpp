#ifndef DSOPP_OUTPUT_PARAMETERS_HPP
#define DSOPP_OUTPUT_PARAMETERS_HPP

namespace dsopp::output {
/** Output parameters received from the user and used to collect the necessary information */
struct OutputParameters {
  /** visualization state for the immature landmarks in the 3d */
  bool show_immature_landmarks = false;
  /** visualization state for the active landmarks in the 3d */
  bool show_active_landmarks = true;
  /** visualization state for the marginalized landmarks in the 3d */
  bool show_marginalized_landmarks = true;
  /** visualization state for the outliers in the 3d */
  bool show_outliers = false;
  /** visualization state for colors */
  bool colored_view = false;
  /** visualization state for track-based unique colors (could be shown only with `colored_view = false` */
  bool track_based_colors = false;
  /** visualization state for semantics colors */
  bool show_semantics_colors = false;
  /** visualization state for the odometry track */
  bool show_odometry_track = true;
  /** visualization state for the GNSS track */
  bool show_gnss_track = false;
  /** visualization state for the camera */
  bool show_camera = true;
  /** toggle of filtering by idepth variance and relative baseline */
  bool apply_filters = false;
  /** visualization state for candidates */
  bool show_candidates = true;
  /** visualization state for failed correspondences */
  bool show_failed = true;
  /** visualization state for relocalized correspondences */
  bool show_relocalized = true;
  /** visualization state for skiped correspondences */
  bool show_skipped = true;
  /** camera size value */
  double camera_size = 1.5;
  /** threshold value of idepth variance */
  double idepth_variance_threshold = 1e-5;
  /** threshold value of relative baseline */
  double relative_baseline_threshold = 0.1;
};
}  // namespace dsopp::output
#endif  // DSOPP_OUTPUT_PARAMETERS_HPP
