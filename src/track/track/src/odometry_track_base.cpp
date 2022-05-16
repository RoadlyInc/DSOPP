#include "track/odometry_track_base.hpp"

#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"

#include "track/connections/connections_container.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
OdometryTrackBase<Motion>::OdometryTrackBase()
    : connections_(std::make_unique<ConnectionsContainer<typename Motion::Product>>()) {}

template <energy::motion::Motion Motion>
OdometryTrackBase<Motion>::OdometryTrackBase(
    std::unique_ptr<ConnectionsContainer<typename Motion::Product>>&& connections)
    : connections_(std::move(connections)) {}

template <energy::motion::Motion Motion>
OdometryTrackBase<Motion>::~OdometryTrackBase() = default;

template <energy::motion::Motion Motion>
ConnectionsContainer<typename Motion::Product>& OdometryTrackBase<Motion>::connections() {
  return *connections_;
}

template class OdometryTrackBase<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
