
#ifndef DSOPP_FRAME_HPP
#define DSOPP_FRAME_HPP

#include <map>
#include <memory>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "track/frames/points_storage.hpp"

namespace dsopp {
namespace track {
/**
 * state of the agent in the given moment
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class Frame {
 public:
  /** exported motion type */
  using MotionT = Motion;
  /**
   * @param id frame id
   * @param timestamp time of the capture
   * @param tWorldAgent pose of the agent
   * @param exposure_time exposure time
   * @param affine_brightness affine brightness
   */
  Frame(size_t id, time timestamp, const Motion &tWorldAgent, const Precision exposure_time,
        const Eigen::Vector<Precision, 2> &affine_brightness);
  /**
   * @return transformation from agent's coordinate system to world's
   */
  const Motion &tWorldAgent() const;
  /**
   * @return time of the frame
   */
  time timestamp() const;
  /**
   * @return id of a frame
   */
  size_t id() const;
  /**
   * @return exposure time
   */
  Precision exposureTime() const;
  /**
   * @return affine brightness
   */
  const Eigen::Vector<Precision, 2> &affineBrightness() const;
  ~Frame();

  /**
   * decodes image from ``image_buffer``
   * @param sensor_id sensor id
   * @return decoded image
   */
  cv::Mat image(const size_t sensor_id) const;

  /**
   * add image to Frame
   * Method would encode image to ``jpg`` buffer
   * @param sensor_id sensor id
   * @param image image
   */
  void pushImage(const size_t sensor_id, const cv::Mat &image);

  /**
   * add encoded image to Frame
   * @param sensor_id sensor id
   * @param image encoded image
   */
  void pushImage(const size_t sensor_id, std::vector<uchar> &&image);

 protected:
  /** id of a frame, number of a frame from the start*/
  const size_t id_;
  /** timestamp of the frame */
  const time timestamp_;
  /** Agent to world transformation */
  Motion tWorldAgent_;
  /** exposure time */
  const Precision exposure_time_;
  /** affine brightness */
  Eigen::Vector<Precision, 2> affine_brightness_;
  /** stores 3-channel image in ``uchar`` buffer
   * ``cv::imencode`` is used to produce it
   * ``cv::imdecode`` should be used to decode
   */
  std::map<unsigned int, std::vector<uchar>> image_buffer_;
};
/**
 * function to convert landmarks from the sensor to the points
 *
 * @param landmarks landmarks from the sensor
 * @param sensor_id sensor
 * @param[out] points_storage vector to add created points
 * @tparam LandmarkType type of the landmark
 */
template <typename LandmarkType, energy::motion::Motion Motion>
void buildPoints(const Motion &t_w_a, const std::vector<LandmarkType> &landmarks, const size_t sensor_id,
                 storage::PointsStorage &points_storage,
                 storage::PointStorageType type = storage::PointStorageType::kActive);

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_FRAME_HPP
