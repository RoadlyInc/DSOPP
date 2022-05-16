#include "feature_based_slam/track/frame.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {

Frame::Frame(const time &_timestamp, const size_t _frame_id, cv::Mat _image)
    : timestamp(_timestamp),
      frame_id(_frame_id),
      t_world_agent(
          energy::motion::SE3<Precision>::exp(Eigen::Vector<Precision, energy::motion::SE3<Precision>::DoF>::Zero())),
      initialized(false),
      image(_image) {}

}  // namespace track
}  // namespace feature_based_slam
}  // namespace dsopp
