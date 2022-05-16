#include "feature_based_slam/features/optical_flow.hpp"
#include "feature_based_slam/features/orb.hpp"

#include <gtest/gtest.h>

#include "common/image_tools/conversion.hpp"
#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "feature_based_slam/features/correspondences_finder.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_extractor_orb.hpp"
#include "test/tools/solver_test_data.hpp"

namespace dsopp::feature_based_slam::features {

TEST(feature_based_slam_features_matcher, feature_based_slam_features_matcher) {
  const double kRatioThreshold = 0.9;

  using SE3 = energy::motion::SE3<Precision>;
  test_tools::SolverTestData<SE3> data({10, 11});
  auto feature_extractor = std::make_unique<DistinctFeaturesExtractorORB>(1000);

  auto reference_image =
      common::image_tools::pixelMap2Mat1C<1>(data.track.odometryTrack().keyframes()[0]->getLevel(data.sensor, 0), 0);
  auto target_image =
      common::image_tools::pixelMap2Mat1C<1>(data.track.odometryTrack().keyframes()[1]->getLevel(data.sensor, 0), 0);

  auto [features_from, _] = findCorrespondences(nullptr, cv::Mat(), reference_image, data.settings->cameraMask(),
                                                *feature_extractor, features::OpticalFlowMatch);
  auto [features_to_optical, correspondences_optical] =
      findCorrespondences(features_from.get(), reference_image, target_image, data.settings->cameraMask(),
                          *feature_extractor, features::OpticalFlowMatch);
  auto [features_to_orb, correspondences_orb] =
      findCorrespondences(features_from.get(), reference_image, target_image, data.settings->cameraMask(),
                          *feature_extractor, features::ORBMatch);

  double inliers = 0;
  double founded = 0;

  for (const auto &optical : correspondences_optical) {
    if (optical.is_inlier) {
      const Correspondence *orb = nullptr;
      for (const auto &orb_candidate : correspondences_orb) {
        if (orb_candidate.idx_from == optical.idx_from) {
          orb = &orb_candidate;
        }
      }

      if (orb && orb->is_inlier) {
        founded++;
        double distance = (features_to_optical->features()[static_cast<size_t>(optical.idx_to)].coordinates() -
                           features_to_orb->features()[static_cast<size_t>(orb->idx_to)].coordinates())
                              .norm();
        if (distance < 2) {
          inliers++;
        }
      }
    }
  }

  ASSERT_GE(inliers / founded, kRatioThreshold);
}

}  // namespace dsopp::feature_based_slam::features
