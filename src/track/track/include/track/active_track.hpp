#ifndef DSOPP_ACTIVE_TRACK_HPP
#define DSOPP_ACTIVE_TRACK_HPP

#include <memory>

#include "track/track_base.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class ActiveOdometryTrack;

template <energy::motion::Motion Motion>
class Track;

/**
 * \brief Active track.
 *
 * This type of track is created using the slam algorithm.
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class ActiveTrack final : public TrackBase<ActiveOdometryTrack, Motion> {
 public:
  ActiveTrack();

  storage::Track serialize() const override;

  ~ActiveTrack() override;

  /**
   * method to create `Track` class instance
   * @return unique ptr to `Track`
   */
  std::unique_ptr<Track<Motion>> createTrack() const;
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_ACTIVE_TRACK_HPP
