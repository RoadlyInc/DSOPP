#include "sensors/camera_calibration/undistorter/undistorter.hpp"
#include "common/settings.hpp"
#include "energy/camera_model/fisheye/atan_camera.hpp"
#include "energy/camera_model/pinhole/ios_camera_model.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

using Precision = dsopp::Precision;
using Undistorter = dsopp::sensors::calibration::Undistorter;
using IOSCamera = dsopp::energy::model::IOSCamera;
using AtanCamera = dsopp::energy::model::AtanCamera;
using PinholeCamera = dsopp::energy::model::PinholeCamera<Precision>;
using SimpleRadial = dsopp::energy::model::SimpleRadialCamera<Precision>;
using dsopp::operator"" _p;

bool insideImage(Precision x, Precision y, const Eigen::Vector2<Precision> &img_size) {
  if (x < 0 || y < 0) return false;
  if (x >= img_size(0) || y >= img_size(1)) return false;
  return true;
}

template <size_t kNumberOfPoints = 3>
std::array<Eigen::Vector3<Precision>, kNumberOfPoints> generatePoints(Precision k, Precision b,
                                                                      const PinholeCamera &phc,
                                                                      const Eigen::Vector2<Precision> &img_size,
                                                                      std::mt19937 &rng,
                                                                      std::uniform_real_distribution<Precision> &x_) {
  std::array<Eigen::Vector2<Precision>, kNumberOfPoints> pts;

  const int kMinDistance = 5;

  for (size_t p = 0; p < kNumberOfPoints;) {
    Precision x = x_(rng);
    Precision y = k * x + b;

    bool point_was_previously_added = false;
    size_t point_idx = 0;
    while (point_idx < p) {
      if (abs(pts[point_idx](0) - x) < kMinDistance) point_was_previously_added = true;
      point_idx++;
    }

    if (point_was_previously_added || !insideImage(x, y, img_size)) continue;

    pts[p](0) = x;
    pts[p](1) = y;

    p++;
  }
  std::array<Eigen::Vector3<Precision>, kNumberOfPoints> rays;
  for (size_t p = 0; p < kNumberOfPoints; ++p) phc.unproject(pts[p], rays[p]);
  return rays;
}

class AtanCameraUndistorterTest : public ::testing::Test {
 public:
  void SetUp() {
    img_size = Eigen::Vector2<Precision>(3040, 3040);
    focals = Eigen::Vector2<Precision>(855.568, 855.839);
    center = Eigen::Vector2<Precision>(1417.72, 1439.66);

    polynomial = Eigen::VectorX<Precision>(8);
    polynomial << 0, 0.0591006_p, 0, -0.0105943_p, 0, -0.00467958_p, 0, 0.000500274_p;

    model = new AtanCamera(img_size, focals, center, polynomial);
  }

  void TearDown() { delete model; }

  bool inside(Precision x, Precision y) {
    if (x < 0 || y < 0) return false;
    if (x >= img_size(0) || y >= img_size(1)) return false;
    return true;
  }

  AtanCamera *model;
  Eigen::Vector2<Precision> img_size;
  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> center;
  Eigen::VectorX<Precision> polynomial;
};

TEST_F(AtanCameraUndistorterTest, ThreePointsOnLineTest) {
  std::mt19937 rng;
  /** y = kx + b */
  std::uniform_real_distribution<Precision> k_(-4, 0);
  std::uniform_real_distribution<Precision> b_(img_size(0) / 5, img_size(0) * 3 / 5);
  std::uniform_real_distribution<Precision> x_(200, img_size(0) - 200);

  dsopp::energy::model::PinholeCamera pinholeCamera(img_size, focals, center);

  auto [pinhole_undistorted, undistorter] =
      dsopp::sensors::calibration::Undistorter::constructRemaps<AtanCamera>(*model);

  const size_t kNumberOfPoints = 3;
  const int kNumberOfTests = 10;

  for (int i = 0; i < kNumberOfTests; ++i) {
    /** Generate line and make 3d points with initial model without use of distortion */
    Precision k = k_(rng);
    Precision b = b_(rng);

    std::array<Eigen::Vector3<Precision>, kNumberOfPoints> rays =
        generatePoints<kNumberOfPoints>(k, b, pinholeCamera, img_size, rng, x_);
    cv::Mat img = cv::Mat::zeros(cv::Size(int(img_size(0)), int(img_size(1))), CV_8UC1);

    std::array<Eigen::Vector2<Precision>, kNumberOfPoints> projected_pts;
    std::array<Eigen::Vector2<Precision>, kNumberOfPoints> pts;

    for (size_t p = 0; p < kNumberOfPoints; ++p) {
      model->project<Precision>(rays[p], projected_pts[p]);

      pinhole_undistorted.project(rays[p], pts[p]);
    }

    for (size_t j = 0; j < kNumberOfPoints; ++j) {
      cv::circle(img, cv::Point(static_cast<int>(projected_pts[j].x()), static_cast<int>(projected_pts[j].y())), 4,
                 cv::Scalar(255), -1);
    }

    cv::Mat dst = undistorter.undistort(img);

    const int kNotBlackPixel = 100;

    for (size_t j = 0; j < kNumberOfPoints; ++j) {
      int col = static_cast<int>(dst.at<unsigned char>(static_cast<int>(pts[j].y()), static_cast<int>(pts[j].x())));
      EXPECT_GT(col, kNotBlackPixel);
    }
  }
}
