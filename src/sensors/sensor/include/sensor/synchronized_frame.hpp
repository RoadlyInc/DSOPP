#ifndef DSOPP_SYNCHRONIZED_FRAME_HPP
#define DSOPP_SYNCHRONIZED_FRAME_HPP

#include <map>
#include <memory>
#include <vector>

#include "energy/motion/se3_motion.hpp"

#include "common/settings.hpp"
#include "common/time/time.hpp"

namespace dsopp {
namespace features {
class CameraFeatures;
class GnssFeatures;
class IMUFeatures;
}  // namespace features
namespace sensors {
/**
 * all frames received from all sensors on one tick
 */
class SynchronizedFrame {
 public:
  /** alias for SE3<Precision> motion type */
  using SE3 = energy::motion::SE3<Precision>;
  /** alias for SE3<double> motion type */
  using SE3d = energy::motion::SE3<double>;
  /**
   * @param id frame id
   * @param timestamp frame timestamp
   */
  SynchronizedFrame(int id, time timestamp);
  /**
   * adds camera features to collection
   * @param id sensors_id
   * @param frame features to add
   */
  void addCameraFeatures(size_t id, std::unique_ptr<features::CameraFeatures>&& frame);
  /**
   * adds gnss features to collection
   * @param id sensors_id
   * @param frame features to add
   */
  void addGnssFeatures(size_t id, std::unique_ptr<features::GnssFeatures>&& frame);
  /**
   * adds IMU features to collection
   * @param id sensors_id
   * @param frame features to add
   */
  void addIMUFeatures(size_t id, std::unique_ptr<features::IMUFeatures>&& frame);
  /**
   * sets reference transformation between our local coordinate system and world coordinate system
   * @param t_world_local reference transformation between our local coordinate system and world coordinate system
   */
  void setTWorldLocal(const SE3d& t_world_local);
  /**
   * @return all camera features
   */
  const std::map<size_t, std::unique_ptr<features::CameraFeatures>>& cameraFeatures() const;
  /**
   * @return reference transformation between our local coordinate system and world coordinate system
   */
  const SE3d& tWorldLocal() const;
  /**
   * @return id
   */
  int id() const;
  /**
   * @return timestamp
   */
  time timestamp() const;

  ~SynchronizedFrame();

 private:
  /** id of the frame */
  int frame_id_;
  /** time of the frame*/
  time timestamp_;
  /** camera features (indexed by sensor ids)*/
  std::map<size_t, std::unique_ptr<features::CameraFeatures>> camera_frames_;
  SE3d t_world_local_;
};
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_SYNCHRONIZED_FRAME_HPP
