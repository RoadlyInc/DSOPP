#ifndef DSOPP_FEATURE_BASED_SLAM_TRACK_TRACK_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACK_TRACK_HPP

#include <deque>

#include "feature_based_slam/track/frame.hpp"
#include "feature_based_slam/track/landmarks.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {
/**
 * \brief Track in feature based slam. Consists of frames and landmarks that are projected onto frames.
 */
struct Track {
  /** all frames in the track */
  std::deque<Frame> frames;
  /** all 3d landmarks in the track */
  Landmarks landmarks;
};
}  // namespace track
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACK_TRACK_HPP
