#include <gtest/gtest.h>

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/frames/frame.hpp"
#include "track/frames/keyframe.hpp"
#include "track/landmarks/tracking_landmark.hpp"

namespace {
cv::Mat testImage() {
  cv::Mat image = cv::imread(TEST_DATA_DIR "rotation_pair/img1.png");

  std::vector<uchar> image_buffer;
  std::string kImageExt = ".jpg";
  std::vector<int> encode_parameters = {cv::IMWRITE_JPEG_QUALITY, 100};
  cv::imencode(kImageExt, image, image_buffer, encode_parameters);

  return cv::imdecode(image_buffer, cv::IMREAD_UNCHANGED);
}

bool equalMatrices(const cv::Mat &a, const cv::Mat &b) {
  cv::Mat diff;
  cv::absdiff(a, b, diff);
  cv::Mat diff_single_channel = diff.reshape(1, 0);
  cv::Scalar mean = cv::mean(diff_single_channel);
  return mean.val[0] < 0.5;
}

}  // namespace

namespace dsopp::track {

TEST(KeyframeTest, EncodeDecodeImage) {
  using Motion = energy::motion::SE3<Precision>;

  Motion tWorldAgent;
  size_t id = 0;
  time timestamp;
  Eigen::Vector<Precision, 2> affine_brightness;
  affine_brightness.setRandom();
  typename Keyframe<Motion>::LandmarksFrame landmarks;
  cv::Mat image = testImage();

  auto frame =
      std::make_unique<Keyframe<Motion>>(id, id, timestamp, tWorldAgent, affine_brightness, std::move(landmarks));
  frame->pushImage(0, image);
  cv::Mat decoded_image = frame->image(0);
  EXPECT_TRUE(equalMatrices(image, decoded_image));

  auto proto = frame->proto();
  Keyframe<Motion> frame_proto(proto);
  cv::Mat decoded_image_proto = frame_proto.image(0);
  EXPECT_TRUE(equalMatrices(image, decoded_image_proto));

  frame->attachTrackingFrame(id, timestamp, tWorldAgent, affine_brightness);
  EXPECT_TRUE(frame->lastAttachedFrame().image(0).empty());

  frame->lastAttachedFrame().pushImage(0, image);
  cv::Mat decoded_attached_image = frame->lastAttachedFrame().image(0);
  EXPECT_TRUE(equalMatrices(image, decoded_attached_image));
}

}  // namespace dsopp::track
