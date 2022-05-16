#include "test/tools/compare_track.hpp"

#include <gtest/gtest.h>

#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/frames/tracking_frame.hpp"

namespace dsopp {
namespace test_tools {
namespace {
bool compareImages(const cv::Mat &image, const cv::Mat &image_reloaded) {
  cv::Mat diff;
  cv::absdiff(image, image_reloaded, diff);
  cv::Mat diff_single_channel = diff.reshape(1, 0);
  cv::Scalar mean = cv::mean(diff_single_channel);
  return mean.val[0] == 0;
}

void compareCameraSettings(const sensors::calibration::CameraSettings &camera_settings,
                           const sensors::calibration::CameraSettings &camera_settings_reloaded) {
  // intrinsics
  auto intrinsics = camera_settings.calibration().cameraIntrinsics();
  auto intrinsics_reloaded = camera_settings_reloaded.calibration().cameraIntrinsics();
  EXPECT_EQ(intrinsics.size(), intrinsics_reloaded.size());
  for (int i = 0; i < intrinsics.size(); ++i) EXPECT_EQ(intrinsics(i), intrinsics_reloaded(i));

  // pcalib
  auto pcalib = camera_settings.photometricCalibration();
  auto pcalib_reloaded = camera_settings_reloaded.photometricCalibration();
  EXPECT_EQ(pcalib.size(), pcalib_reloaded.size());
  for (size_t i = 0; i < pcalib.size(); ++i) EXPECT_EQ(pcalib[i], pcalib_reloaded[i]);

  // vignetting
  EXPECT_TRUE(compareImages(camera_settings.vignetting(), camera_settings_reloaded.vignetting()));

  // camera mask
  EXPECT_TRUE(compareImages(camera_settings.cameraMask().data(), camera_settings_reloaded.cameraMask().data()));

  // model type
  EXPECT_EQ(camera_settings.calibration().type(), camera_settings_reloaded.calibration().type());

  // image size
  auto image_size = camera_settings.calibration().image_size();
  auto image_size_reloaded = camera_settings_reloaded.calibration().image_size();
  EXPECT_EQ(image_size(0), image_size_reloaded(0));
  EXPECT_EQ(image_size(1), image_size_reloaded(1));

  // shutter time
  EXPECT_EQ(camera_settings.calibration().shutterTime(), camera_settings_reloaded.calibration().shutterTime());

  // legend
  EXPECT_TRUE(camera_settings.semanticLegend() && camera_settings_reloaded.semanticLegend());
  const auto &tags = camera_settings.semanticLegend()->tags();
  const auto &tags_reloaded = camera_settings_reloaded.semanticLegend()->tags();
  EXPECT_EQ(tags.size(), tags_reloaded.size());
  for (size_t i = 0; i < tags.size(); ++i) {
    const auto &tag = tags[i];
    const auto &tag_reloaded = tags_reloaded[i];
    EXPECT_EQ(tag.code, tag_reloaded.code);
    EXPECT_EQ(tag.name, tag_reloaded.name);
    EXPECT_EQ(tag.weight, tag_reloaded.weight);
  }
}

template <energy::motion::Motion Motion>
void compareOdometryTrack(track::OdometryTrack<Motion> &odometry_track,
                          track::OdometryTrack<Motion> &odometry_track_reloaded, const Precision kEps) {
  auto frames1 = odometry_track.keyframes();
  auto frames2 = odometry_track_reloaded.keyframes();
  ASSERT_EQ(frames1.size(), frames2.size());
  for (size_t frame_iter = 0; frame_iter < frames1.size(); frame_iter++) {
    const auto &frame1 = frames1[frame_iter];
    const auto &frame2 = frames2[frame_iter];
    ASSERT_EQ(frame1->id(), frame2->id());
    ASSERT_EQ(frame1->keyframeId(), frame2->keyframeId());
    ASSERT_TRUE(frame1->timestamp() == frame2->timestamp());
    ASSERT_TRUE((frame1->tWorldAgent() * frame2->tWorldAgent().inverse()).log().norm() < kEps);
    ASSERT_TRUE((frame1->affineBrightness() - frame2->affineBrightness()).norm() < kEps);
    const auto &attached_frames1 = frame1->attachedFrames();
    const auto &attached_frames2 = frame2->attachedFrames();
    ASSERT_EQ(attached_frames1.size(), attached_frames2.size());
    for (size_t attached_frame_iter = 0; attached_frame_iter < attached_frames1.size(); attached_frame_iter++) {
      const auto &attached_frame1 = attached_frames1[attached_frame_iter];
      const auto &attached_frame2 = attached_frames2[attached_frame_iter];
      ASSERT_TRUE(attached_frame1->timestamp() == attached_frame2->timestamp());
      ASSERT_TRUE((attached_frame1->tWorldAgent() * attached_frame2->tWorldAgent().inverse()).log().norm() < kEps);
      ASSERT_TRUE(attached_frame1->affineBrightness() == attached_frame2->affineBrightness());
    }
    const auto connections1 = odometry_track.connections().get(frame_iter);
    const auto connections2 = odometry_track_reloaded.connections().get(frame_iter);
    ASSERT_EQ(connections1.size(), connections2.size());
    for (size_t connection_iter = 0; connection_iter < connections1.size(); connection_iter++) {
      ASSERT_EQ(connections1[connection_iter]->referenceKeyframeId(),
                connections2[connection_iter]->referenceKeyframeId());
      ASSERT_EQ(connections1[connection_iter]->targetKeyframeId(), connections2[connection_iter]->targetKeyframeId());
      ASSERT_TRUE((connections1[connection_iter]->covariance() - connections2[connection_iter]->covariance()).norm() <
                  kEps);
    }
  }
}

void compareSanityChecks(const std::map<size_t, sanity_checker::SanityCheckStatus> &sanity_check_results,
                         const std::map<size_t, sanity_checker::SanityCheckStatus> &sanity_check_results_reloaded) {
  ASSERT_EQ(sanity_check_results, sanity_check_results_reloaded);
}

void compareAgentSettings(const sensors::AgentSettings &agent_settings,
                          const sensors::AgentSettings &agent_settings_reloaded) {
  EXPECT_EQ(agent_settings.cameraSettings().size(), agent_settings_reloaded.cameraSettings().size());

  for (auto &[id, settings] : agent_settings.cameraSettings()) {
    EXPECT_TRUE(agent_settings_reloaded.cameraSettings().contains(id));
    compareCameraSettings(settings, agent_settings_reloaded.cameraSettings().at(id));
  }
}
}  // namespace

template <energy::motion::Motion Motion>
void compareTrack(const track::Track<Motion> &track, const track::Track<Motion> &track_reloaded, const Precision kEps) {
  auto &agent_settings = track.agentSettings();
  auto &odometry_track = track.odometryTrack();
  auto &sanity_check_results = track.sanityCheckResults();

  auto &agent_settings_reloaded = track_reloaded.agentSettings();
  auto &odometry_track_reloaded = track_reloaded.odometryTrack();
  auto &sanity_check_results_reloaded = track_reloaded.sanityCheckResults();

  compareOdometryTrack(odometry_track, odometry_track_reloaded, kEps);
  compareSanityChecks(sanity_check_results, sanity_check_results_reloaded);
  compareAgentSettings(agent_settings, agent_settings_reloaded);
}

template void compareTrack(const track::Track<energy::motion::SE3<Precision>> &,
                           const track::Track<energy::motion::SE3<Precision>> &, const Precision);
}  // namespace test_tools
}  // namespace dsopp
