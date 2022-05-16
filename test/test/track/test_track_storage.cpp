#include <gtest/gtest.h>

#include "storage/track_storage.hpp"
#include "test/tools/compare_track.hpp"
#include "test/tools/random_track.hpp"

namespace dsopp {
namespace track {
TEST(testStorage, testStorage) {
  using SE3 = energy::motion::SE3<Precision>;
  const Precision kEps = 1e-14_p;

  auto track = test_tools::track();

  auto track_reloaded = Track<SE3>(track->serialize());

  test_tools::compareTrack(*track, track_reloaded, kEps);
}
}  // namespace track
}  // namespace dsopp
