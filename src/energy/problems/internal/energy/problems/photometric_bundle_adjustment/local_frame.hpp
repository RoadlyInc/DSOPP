#ifndef DSOPP_PHOTOMETRIC_BUNDLE_ADJUSTMENT_LOCAL_FRAME_HPP
#define DSOPP_PHOTOMETRIC_BUNDLE_ADJUSTMENT_LOCAL_FRAME_HPP

#include <memory>

#include "common/patch/patch.hpp"
#include "common/pattern/pattern.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/problems/depth_map.hpp"
#include "energy/problems/photometric_bundle_adjustment/frame_parameterization.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_map.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/tracking_landmark.hpp"
#include "track/landmarks/tracking_landmark_with_patch.hpp"

namespace dsopp {
namespace energy {
namespace problem {

struct DepthMap;
/**
 * Traits to have same interface with different grids
 *
 * @tparam Grid2D interpolation grid
 * @tparam C number of channels in grid
 */
template <template <int> typename Grid2D, int C>
struct GridTraits;

/**
 * Traits for PixelMap
 *
 * @tparam C number of channels in grid
 */
template <int C>
struct GridTraits<features::PixelMap, C> {
  /** shortcut to the PixelMap ptr */
  using grid_ptr = const features::PixelMap<C> *;
  /** create grid from the PixelMap
   *
   * @param pixel_map pixel map
   * @return pixel map
   */
  static grid_ptr getGrid(const features::PixelMap<C> *pixel_map) { return pixel_map; }
  /** get the ptr from the PixelMap
   *
   * @param pixel_map pixel map
   * @return pixel map
   */
  static grid_ptr ptr(const features::PixelMap<C> *pixel_map) { return pixel_map; }
};

/**
 * Traits for CeresGrid
 *
 * @tparam C number of channels in grid
 */
template <int C>
struct GridTraits<features::CeresGrid, C> {
  /** shortcut to the CeresGrid ptr */
  using grid_ptr = std::unique_ptr<features::CeresGrid<C>>;
  /** create grid from the PixelMap
   *
   * @param pixel_map pixel map
   * @return ceres grid
   */
  static grid_ptr getGrid(const features::PixelMap<C> *pixel_map) {
    return std::make_unique<features::CeresGrid<C>>(*pixel_map);
  }
  /** get the ptr from the CeresGrid
   *
   * @param ceres_grid ceres grid
   * @return ceres grid ptr
   */
  static const features::CeresGrid<C> *ptr(const std::unique_ptr<features::CeresGrid<C>> &ceres_grid) {
    return ceres_grid.get();
  }
};

template <typename Scalar, int PatternSize, int C>
Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> getPatch(
    const Eigen::Matrix<Precision, Pattern::kSize, C, PatchStorageOrder<C>> &patch) {
  Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> result_patch;
  if constexpr (PatternSize == Pattern::kSize) {
    result_patch = patch.template cast<Scalar>();
  }
  if constexpr (PatternSize == 1) {
    result_patch = patch.row(Pattern::kCenter).template cast<Scalar>();
  }
  return result_patch;
}  // namespace

/**
 * Traits to have same landmark interface with different numbers of channels
 *
 * @tparam C number of channels
 */
template <int C>
struct LandmarkTraits {
  /**
   * @throw invalid argument exception
   * @return nothing
   */
  static const Eigen::Matrix<Precision, Pattern::kSize, C, PatchStorageOrder<C>> &patch(
      const track::landmarks::TrackingLandmarkWithPatch &) {
    throw std::invalid_argument("Wrong number of channels in LandmarkTraits::patch");
  }
};

/**
 * Traits for C = 1
 */
template <>
struct LandmarkTraits<1> {
  /** get patch of the landmark
   *
   * @param landmark landmark
   * @return patch
   */
  static const Eigen::Vector<Precision, Pattern::kSize> &patch(
      const track::landmarks::TrackingLandmarkWithPatch &landmark) {
    return landmark.patch();
  }
};

/**
 * Traits to have same pyramids interface with different numbers of channels
 *
 * @tparam Motion Motion type
 * @tparam C number of channels in pyramids
 */
template <energy::motion::Motion Motion, int C>
struct PyramidsTraits {
  /**
   * @throw invalid argument exception
   * @return nothing
   */
  static const features::PixelMap<C> &getLevel(const track::ActiveKeyframe<Motion> &, const size_t, size_t) {
    throw std::invalid_argument("Wrong number of channels in PyramidsTraits::getLevel");
  };
};

/**
 * Traits for C = 1
 *
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
struct PyramidsTraits<Motion, 1> {
  /** get pyramid level of the frame data
   *
   * @param frame frame
   * @param sensor id of the sensor
   * @param level level to get
   * @return pyramid level
   */
  static const features::PixelMap<1> &getLevel(const track::ActiveKeyframe<Motion> &frame, const size_t sensor,
                                               size_t level) {
    return frame.getLevel(sensor, level);
  }
};

/**
 * Residual point for one landmark between two frames
 * @tparam Motion Motion type
 */
template <typename Scalar, energy::motion::Motion Motion, int PatternSize, int C>
struct ResidualPoint {
  /** connection status */
  track::PointConnectionStatus connection_status;
  /** connection status candidate */
  track::PointConnectionStatus connection_status_candidate;
  /** current residual */
  Eigen::Vector<Scalar, PatternSize * C> residuals;
  /** derivative of u coordinate with respect to idepth*/
  Eigen::Vector<Scalar, PatternSize> d_u_idepth;
  /** derivative of v coordinate with respect to idepth*/
  Eigen::Vector<Scalar, PatternSize> d_v_idepth;
  /** derivative of u coordinate with respect to target to reference transformation*/
  Eigen::Matrix<Scalar, PatternSize, Motion::Product::DoF> d_u_tReferenceTarget;
  /** derivative of u coordinate with respect to target to reference transformation*/
  Eigen::Matrix<Scalar, PatternSize, Motion::Product::DoF> d_v_tReferenceTarget;
  /** true if jacobians of reprojection (4 line above) were evaluated, otherwise  jacobians in this structure are
   * uninitialized */
  bool reprojection_jacobians_valid = false;
  /** true if residual was estimated at least once, marginalized points which were estimated while being active are not
   * being recalculatied*/
  bool was_estimated = false;
  /** current reference state differential */
  Eigen::Matrix<Scalar, PatternSize * C, Motion::DoF + 2, Eigen::RowMajor> d_reference_state_eps;
  /** current target state differential */
  Eigen::Matrix<Scalar, PatternSize * C, Motion::DoF + 2, Eigen::RowMajor> d_target_state_eps;
  /** current idepth differential */
  Eigen::Matrix<Scalar, PatternSize * C, 1> d_idepth;
  /** weight for huber loss*/
  Scalar huber_weight = 1;
  /** energy of the residual*/
  Scalar energy = 0;
  /** brightness change evaluated at the linearization point*/
  Scalar brightness_change_scale = 0;
  /**
   * Creates residual point for the given landmark.
   *
   * @param _connection_status connection status
   */
  ResidualPoint(track::PointConnectionStatus _connection_status = track::PointConnectionStatus::kOk)
      : connection_status(_connection_status), connection_status_candidate(_connection_status) {
    d_reference_state_eps.setZero();
    d_target_state_eps.setZero();
    d_idepth.setZero();
    residuals.setZero();
    huber_weight = 1;
  }
};

/** \brief Local copy of the frame for the optimizing.
 *
 * Whenever a frame needs to be optimized, a local copy is created. After optimization,
 * the local copy can be copied back to the frame.
 * @tparam Motion motion type
 * @tparam Grid2D 2d optimization grid
 * @tparam C number of channels in the grid
 */
template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, int C>
class LocalFrame {
 public:
  /** frontend reference virtual frame id */
  static constexpr int kFrontendReferenceFrameId = -2;
  /** frontend target virtual frame id */
  static constexpr int kFrontendTargetFrameId = -1;

  /** shortcut for data from sensors */
  using Pyramids = std::map<size_t, std::vector<features::PixelMap<C>>>;
  /** shortcut for masks from sensors */
  using PyramidsOfMasks = typename track::ActiveKeyframe<Motion>::PyramidsOfMasks;
  /** struct which stores information from the landmark */
  struct Landmark {
    /**
     *
     * @param _projection projection of the landmark
     * @param _idepth inverse depth of the landmark
     * @param _patch  patch of the landmark
     * @param _is_marginalized true if marginalize
     * @param _is_outlier  true if outlier
     */
    Landmark(const Eigen::Vector2<Scalar> &_projection, Scalar _idepth,
             const Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> &_patch, bool _is_marginalized,
             bool _is_outlier)
        : projection(_projection),
          idepth(_idepth),
          patch(_patch),
          is_marginalized(_is_marginalized),
          is_outlier(_is_outlier) {
      if constexpr (PatternSize == Pattern::kSize) {
        features::PatternPatch::shiftPattern(projection, reference_pattern);
      }
      if constexpr (PatternSize == 1) {
        reference_pattern = projection;
      }
    }
    /** projection of the landmark */
    const Eigen::Vector2<Scalar> projection;
    /** idepth of the landmark */
    Scalar idepth;
    /** candidate step for idepth*/
    Scalar idepth_step = 0;
    /** patch of the landmark */
    const Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> patch;
    /** true if marginalized */
    bool is_marginalized;
    /** landmark is flagged for marginalization, solver will calculate required linear subsystems in the next solve call
     * and mark it as marginalized
     */
    bool to_marginalize = false;
    /** true if outlier */
    bool is_outlier;
    /** reference pattern of the landmark */
    Eigen::Matrix<Scalar, 2, PatternSize> reference_pattern;
    /** patch corrected with affine brightness tranform*/
    Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> corrected_intensities;
    /** baseline normalized with the idepth */
    Scalar relative_baseline = 0;
    /** inversed hessian of inverse depth from last iteration */
    Scalar inv_hessian_idepth_idepth = 0;
    /** true if landmark's position is ill conditioned */
    bool ill_conditioned = false;
    /** for details please refer to eigen_photometric_bundle_adjustment.cpp  */
    Scalar b_idepth_block = 0;
    /** for details please refer to eigen_photometric_bundle_adjustment.cpp  */
    Eigen::VectorX<Scalar> hessian_poses_idepth_block;
    /** number of inlier residuals in the last optimization */
    size_t number_of_inlier_residuals = 0;
  };
  /**
   * Constructor to create local copy of the frame
   *
   * @param frame frame
   * @param level level from which features are selected
   * @param _model camera model for frame
   * @param _frame_parameterization constraint on frame position
   */
  LocalFrame(const track::ActiveKeyframe<Motion> &frame, size_t level, const Model &_model,
             FrameParameterization _frame_parameterization)
      : timestamp(frame.timestamp()),
        T_w_agent_linearization_point(frame.tWorldAgent().template cast<Scalar>()),
        exposure_time(static_cast<Scalar>(frame.exposureTime())),
        affine_brightness0(frame.affineBrightness().template cast<Scalar>()),
        model(_model),
        intrinsic_parameters(_model.intrinsicsParameters().template cast<Scalar>()),
        state_eps(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()),
        state_eps_step(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()) {
    is_marginalized = frame.isMarginalized();
    id = static_cast<int>(frame.keyframeId());
    frame_parameterization = _frame_parameterization;
    for (const auto &sensor : frame.sensors()) {
      masks.insert({sensor, frame.getMask(sensor, level)});
      if (not frame.isMarginalized()) {
        grids[sensor] = GridTraits<Grid2D, C>::getGrid(&PyramidsTraits<Motion, C>::getLevel(frame, sensor, level));
      }
      active_landmarks[sensor].reserve(frame.activeLandmarks(sensor).size());
      for (const auto &landmark : frame.activeLandmarks(sensor)) {
        active_landmarks[sensor].emplace_back(
            Landmark{landmark.projection().template cast<Scalar>(), static_cast<Scalar>(landmark.idepth()),
                     getPatch<Scalar, PatternSize, C>(LandmarkTraits<C>::patch(landmark)), landmark.isMarginalized(),
                     landmark.isOutlier()});
      }
    }
  }
  /**
   * Constructor to create local frame from the time, transformation and depth map
   *
   * @param _timestamp timestamp of the frame
   * @param _t_world_agent initial pose of the frame
   * @param pyramids pyramids of the data from sensors
   * @param _masks masks from the sensors
   * @param depths_maps depth maps for each sensor
   * @param _exposure_time exposure time
   * @param affine_brightness affine brightness
   * @param level level from which features are selected
   * @param _model camera model for frame
   * @param _frame_parameterization constraint on frame position
   */
  LocalFrame(time _timestamp, const Motion &_t_world_agent, const Pyramids &pyramids,
             const std::map<size_t, const sensors::calibration::CameraMask &> &_masks,
             const std::map<size_t, std::vector<DepthMap>> &depths_maps, const Scalar _exposure_time,
             const Eigen::Vector2<Scalar> &affine_brightness, size_t level, const Model &_model,
             FrameParameterization _frame_parameterization)
      : id(kFrontendReferenceFrameId),
        timestamp(_timestamp),
        T_w_agent_linearization_point(_t_world_agent.template cast<Scalar>()),
        exposure_time(_exposure_time),
        affine_brightness0(affine_brightness),
        model(_model),
        intrinsic_parameters(_model.intrinsicsParameters().template cast<Scalar>()),
        state_eps(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()),
        state_eps_step(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()),
        masks(_masks),
        is_marginalized(true),
        frame_parameterization(_frame_parameterization) {
    static const int kBorderSize = 4;
    static const Scalar kMinIdepth = Scalar(1e-6);

    for (const auto &[sensor, sensor_depths_maps] : depths_maps) {
      grids[sensor] = GridTraits<Grid2D, C>::getGrid(&(pyramids.at(sensor))[level]);

      auto &depth_map = depths_maps.at(sensor).at(level).map;

      int width = static_cast<int>(depth_map.rows()), height = static_cast<int>(depth_map.cols());

      for (int y = kBorderSize; y < height - kBorderSize; y++) {
        for (int x = kBorderSize; x < width - kBorderSize; x++) {
          if (depth_map(x, y).weight > 0) {
            Scalar idepth = static_cast<Scalar>(depth_map(x, y).idepth / depth_map(x, y).weight);
            if (idepth < kMinIdepth) {
              continue;
            }
            Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>> pixel_patch;
            features::PatternPatch::getIntensities(Eigen::Vector2<Precision>(x, y), (pyramids.at(sensor))[level],
                                                   pixel_patch);
            active_landmarks[sensor].emplace_back(
                Landmark{Eigen::Vector2<Scalar>(x, y), idepth, pixel_patch.template cast<Scalar>(), false, false});
          }
        }
      }
    }
  }

  /**
   * Constructor to create local frame from the time, transformation and vector of tracking landmarks
   *
   * @param _timestamp timestamp of the frame
   * @param _t_world_agent initial pose of the frame
   * @param pyramids pyramids of the data from sensors
   * @param _masks masks from the sensors
   * @param _model model
   * @param tracking_landmarks tracking landmarks
   * @param _exposure_time exposure time
   * @param affine_brightness affine brightness
   * @param level level from which features are selected
   * @param level_resize_ratio resize ratio of level image
   * @param _frame_parameterization constraint on frame position
   */
  LocalFrame(time _timestamp, const Motion &_t_world_agent, const Pyramids &pyramids,
             const std::map<size_t, const sensors::calibration::CameraMask &> &_masks,
             const std::map<size_t, std::vector<track::landmarks::TrackingLandmark>> &tracking_landmarks,
             const Scalar _exposure_time, const Eigen::Vector2<Scalar> &affine_brightness, const Model &_model,
             size_t level, Precision level_resize_ratio, FrameParameterization _frame_parameterization)
      : id(kFrontendReferenceFrameId),
        timestamp(_timestamp),
        T_w_agent_linearization_point(_t_world_agent.template cast<Scalar>()),
        exposure_time(_exposure_time),
        affine_brightness0(affine_brightness),
        model(_model),
        state_eps(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()),
        state_eps_step(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()),
        masks(_masks),
        is_marginalized(false),
        frame_parameterization(_frame_parameterization) {
    for (const auto &pyramid : pyramids) {
      const auto &sensor = pyramid.first;
      grids[sensor] = GridTraits<Grid2D, C>::getGrid(&(pyramid.second)[level]);

      for (auto &landmark : tracking_landmarks.at(sensor)) {
        Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>> pixel_patch;
        Eigen::Vector<Precision, 2> xy = landmark.projection() / level_resize_ratio;
        if (!model.insideCameraROI(xy)) continue;

        features::PatternPatch::getIntensities(xy, (pyramids.at(sensor))[level], pixel_patch);
        active_landmarks[sensor].emplace_back(Landmark{xy.template cast<Scalar>(),
                                                       static_cast<Scalar>(landmark.idepth()),
                                                       pixel_patch.template cast<Scalar>(), false, false});
      }
    }
  }

  /**
   * Constructor to create local frame from the time, transformation and pyramids
   *
   * @param _timestamp timestamp of the frame
   * @param _t_world_agent pose of the frame
   * @param pyramids pyramids of the data from sensors
   * @param _masks masks from the sensors
   * @param _exposure_time exposure time
   * @param affine_brightness affine brightness
   * @param _is_marginalized true if marginalized
   * @param level level from which features are selected
   * @param _model camera model for frame
   * @param _frame_parameterization constraint on frame position
   */
  LocalFrame(time _timestamp, const Motion &_t_world_agent, const Pyramids &pyramids,
             const std::map<size_t, const sensors::calibration::CameraMask &> &_masks, const Scalar _exposure_time,
             const Eigen::Vector2<Scalar> &affine_brightness, bool _is_marginalized, size_t level, const Model &_model,
             FrameParameterization _frame_parameterization)
      : id(kFrontendTargetFrameId),
        timestamp(_timestamp),
        T_w_agent_linearization_point(_t_world_agent.template cast<Scalar>()),
        exposure_time(_exposure_time),
        affine_brightness0(affine_brightness),
        model(_model),
        intrinsic_parameters(_model.intrinsicsParameters().template cast<Scalar>()),
        state_eps(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()),
        state_eps_step(Eigen::Vector<Scalar, Motion::DoF + 2>::Zero()),
        masks(_masks),
        is_marginalized(_is_marginalized),
        frame_parameterization(_frame_parameterization) {
    for (const auto &pyramid : pyramids) {
      const auto &sensor = pyramid.first;
      grids[sensor] = std::move(GridTraits<Grid2D, C>::getGrid(&(pyramid.second)[level]));
    }
  }
  /**
   * update information in local frame
   *
   * @param frame frame to get updates from
   * @param connections connections of the frame
   */
  void update(const track::ActiveKeyframe<Motion> &frame,
              std::map<size_t, track::FrameConnection<typename Motion::Product> *> connections) {
    for (const auto &sensor : frame.sensors()) {
      active_landmarks[sensor].reserve(frame.activeLandmarks(sensor).size());

      size_t old_landmarks_size = active_landmarks[sensor].size();
      const auto &landmarks = frame.activeLandmarks(sensor);
      // update statuses of known landmarks
      for (size_t landmark_iter = 0; landmark_iter < old_landmarks_size; landmark_iter++) {
        active_landmarks[sensor][landmark_iter].to_marginalize =
            !active_landmarks[sensor][landmark_iter].is_marginalized && landmarks[landmark_iter].isMarginalized() &&
            !landmarks[landmark_iter].isOutlier();
        active_landmarks[sensor][landmark_iter].is_marginalized = landmarks[landmark_iter].isMarginalized();
      }
      // add freshly matured landmarks
      for (size_t landmark_iter = old_landmarks_size; landmark_iter < landmarks.size(); landmark_iter++) {
        auto &landmark = landmarks[landmark_iter];
        active_landmarks[sensor].emplace_back(
            Landmark{landmark.projection().template cast<Scalar>(), static_cast<Scalar>(landmark.idepth()),
                     getPatch<Scalar, PatternSize, C>(LandmarkTraits<C>::patch(landmark)), landmark.isMarginalized(),
                     landmark.isOutlier()});
      }
      // add reprojection statuses for matured frames
      for (const auto &[target_id, connection] : connections) {
        for (const auto &pair : connection->getSensorPairs()) {
          auto statuses = &connection->referenceReprojectionStatuses(pair.first, pair.second);
          if (static_cast<int>(target_id) < id) {
            statuses = &connection->targetReprojectionStatuses(pair.first, pair.second);
          }
          for (size_t landmark_iter = residuals[pair][static_cast<int>(target_id)].size();
               landmark_iter < statuses->size(); landmark_iter++) {
            residuals[pair][static_cast<int>(target_id)].push_back(
                ResidualPoint<Scalar, Motion, PatternSize, C>((*statuses)[landmark_iter]));
          }
        }
      }
    }
  }
  /**
   * @return current tranformation of the agent
   */
  typename Motion::template CastT<Scalar> tWorldAgent() const {
    return T_w_agent_linearization_point.rightIncrement(state_eps.template head<Motion::DoF>());
  }
  /**
   * @return current affine brightness
   */
  Eigen::Vector2<Scalar> affineBrightness() const { return affine_brightness0 + state_eps.template tail<2>(); }
  /**
   * @return all sensors
   * */
  std::vector<size_t> sensors() const {
    std::vector<size_t> sensors;
    for (const auto &[key, value] : active_landmarks) {
      sensors.push_back(key);
    }
    return sensors;
  }
  /** frame id */
  int id;
  /** timestamp of the frame */
  time timestamp;
  /** Agent to world transformation linearization point is point which we use to draw a tangent space to replace
   * the target function with linear aproximation*/
  typename Motion::template CastT<Scalar> T_w_agent_linearization_point;
  /** exposure time */
  const Scalar exposure_time;
  /** affine brightness before optimization*/
  Eigen::Vector2<Scalar> affine_brightness0;
  /** camera model for the frame*/
  const Model model;
  /** *intrinsic parameters of camera model */
  Eigen::Vector<Scalar, Model::DoF> intrinsic_parameters;
  /** optimized value linearized around stacked T_w_agent_linearization_point and affine_brightness0
   * first 6 element is an additive value to T_w_agent_linearization_point and last 2 elements is an additive value to
   * affine_brightness0
   */
  Eigen::Matrix<Scalar, Motion::DoF + 2, 1> state_eps;
  /** candidate step to add to state_eps*/
  Eigen::Matrix<Scalar, Motion::DoF + 2, 1> state_eps_step;
  /** map which stores pointers to the PixelMaps from different sensors */
  std::map<size_t, typename GridTraits<Grid2D, C>::grid_ptr> grids;
  /** camera mask to filter projection */
  std::map<size_t, const sensors::calibration::CameraMask &> masks;
  /** true if the local copy is the copy of the marginalized frame */
  bool is_marginalized;
  /** frame is flagged for marginalization, solver will calculate required linear subsystems in the next solve call
   * and mark it as marginalized
   */
  bool to_marginalize = false;
  /** frame_parameterization constraint on frame position */
  FrameParameterization frame_parameterization;
  /** map which stores active landmarks from different sensors */
  std::map<size_t, std::vector<Landmark>> active_landmarks;
  /** frame reprojetion statuses, indexed as [reference sensor id][target sensor id][target frame id][reference point
   * id] */
  std::map<std::pair<size_t, size_t>, std::map<int, std::vector<ResidualPoint<Scalar, Motion, PatternSize, C>>>>
      residuals;
  /** uncertainty matrices of relative transformations */
  std::map<int, Eigen::Matrix<Scalar, Motion::Product::DoF, Motion::Product::DoF>> covariance_matrices;
};

}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_PHOTOMETRIC_BUNDLE_ADJUSTMENT_LOCAL_FRAME_HPP
