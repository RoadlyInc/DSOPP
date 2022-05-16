#ifndef DSOPP_FRAME_CONNECTION_HPP
#define DSOPP_FRAME_CONNECTION_HPP

#include "connection.pb.h"
#include "energy/motion/motion.hpp"

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp {
namespace track {
/**
 * status of a reprojected point
 */
enum struct PointConnectionStatus : uint8_t {
  kOk = 0,      /**< reprojected successfully. */
  kOutlier = 1, /**< energy is too high, marked as outlier (most likely occluded by a static object). */
  kOccluded = 2 /**< occluded by a dynamic object. */,
  kOOB = 3,     /**< reprojected to invalid region of the camera (out of bounds or on static mask). */
  kUnknown = 4, /**< connection added after landmark was marginalized*/
};
/**
 * structure storing relative errors estimation for 2 frames
 *
 * @tparam Motion Motion type
 */
template <energy::motion::MotionProduct MotionProduct>
class FrameConnection {
 public:
  /** reprojections statuses indexed by landmark id*/
  using ReprojectionStatuses = std::vector<PointConnectionStatus>;
  /** reprojections from refernce to target and from target to reference*/
  using SensorConnection = std::pair<ReprojectionStatuses, ReprojectionStatuses>;
  /**
   * creates a frame connection from a given pair
   * @param reference_keyframe_id,target_keyframe_id ids of added pair
   */
  FrameConnection(size_t reference_keyframe_id, size_t target_keyframe_id);
  /**
   * creates a frame connection from proto container
   * @param proto proto container
   */
  FrameConnection(const proto::Connection& proto);
  /**
   * @return reference keyframe id
   */
  size_t referenceKeyframeId() const;
  /**
   * @return target keyframe id
   */
  size_t targetKeyframeId() const;
  /**
   * @param covariance covarince matrix of the t_r_t transformation
   */
  void setCovariance(const Eigen::Matrix<Precision, MotionProduct::DoF, MotionProduct::DoF>& covariance);
  /**
   * @return covarince matrix of the t_r_t transformation
   */
  const Eigen::Matrix<Precision, MotionProduct::DoF, MotionProduct::DoF>& covariance() const;
  /**
   * adds frame connection and sets all reprojection to valid state
   * @param reference_sensor_id, target_sensor_id sensor ids
   * @param reference_landmarks_size, target_landmarks_size number of ladnmarks in reference and target frames
   */
  void addSensorConnection(size_t reference_sensor_id, size_t target_sensor_id, size_t reference_landmarks_size,
                           size_t target_landmarks_size);
  /**
   * @param reference_sensor_id, target_sensor_id  ids of frames
   * @return statuses of reprojections from reference to target frame
   */
  const ReprojectionStatuses& referenceReprojectionStatuses(size_t reference_sensor_id, size_t target_sensor_id) const;
  /**
   * @param reference_sensor_id, target_sensor_id  ids of frames
   * @return statuses of reprojections from target to reference frame
   */
  const ReprojectionStatuses& targetReprojectionStatuses(size_t reference_sensor_id, size_t target_sensor_id) const;
  /**
   * sets statuses of reprojections from reference to target frame
   * @param reference_sensor_id, target_sensor_id  ids of frames
   * @param new_statuses new statuses
   */
  void setReferenceReprojectionStatuses(size_t reference_sensor_id, size_t target_sensor_id,
                                        const ReprojectionStatuses& new_statuses);
  /**
   * sets statuses of reprojections from target to reference frame
   * @param reference_sensor_id, target_sensor_id  ids of frames
   * @param new_statuses new statuses
   */
  void setTargetReprojectionStatuses(size_t reference_sensor_id, size_t target_sensor_id,
                                     const ReprojectionStatuses& new_statuses);
  /**
   * @return all sensors pairs for which point statuses are avaliable
   */
  std::vector<std::pair<size_t, size_t>> getSensorPairs() const;
  /**
   * creates protobuf container from the object
   * @return protobuf container
   */
  proto::Connection proto() const;

 private:
  /** id of a reference frame */
  const size_t reference_keyframe_id_;
  /** id of a target frame */
  const size_t target_keyframe_id_;
  /** covariance matrix of the target to reference transformation */
  Eigen::Matrix<Precision, MotionProduct::DoF, MotionProduct::DoF> t_r_t_covariance_;
  /** point connections status */
  std::map<std::pair<size_t, size_t>, SensorConnection> reprojections_;
};
}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_FRAME_CONNECTION_HPP
