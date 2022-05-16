#ifndef DSOPP_SRC_IO_INTERFACES_IO_INTERFACE_HPP_
#define DSOPP_SRC_IO_INTERFACES_IO_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "energy/motion/motion.hpp"

namespace dsopp {
namespace track {
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class TrackBase;
}  // namespace track
namespace sensors {
class AgentSettings;
}
namespace output {
/**
 * \brief Output interface.
 *
 * Any output of information to external sources should implement this interface.
 * An external source can be, for example, a visualizer or a file.
 */
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class TrackOutputInterface {
 public:
  /**
   * notify that some frames changed in the track
   * @param track changed track
   */
  virtual void notify(const track::TrackBase<OdometryTrackType, Motion> &track) = 0;
  /**
   * notify that track will no longer be changed
   * @param track changed track
   */
  virtual void finish(const track::TrackBase<OdometryTrackType, Motion> &track) { this->notify(track); };
  virtual ~TrackOutputInterface() = default;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_SRC_IO_INTERFACES_IO_INTERFACE_HPP_
