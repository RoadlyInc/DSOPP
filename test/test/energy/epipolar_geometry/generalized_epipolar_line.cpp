#include <fstream>
#include <numbers>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/epipolar_geometry/epipolar_line.hpp"
#include "energy/epipolar_geometry/epipolar_line_builder.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/projector/camera_projector.hpp"
#include "energy/projector/camera_reproject.hpp"

#define DRAW_EPIPOLAR_LINES COMPILE_HIDDEN_CODE

using Precision = dsopp::Precision;
using namespace dsopp::energy;
using namespace dsopp::energy::epipolar_geometry;
using dsopp::operator"" _p;

class testEpipolarLine : public ::testing::Test {
 public:
  void SetUp() override {
    /** to be sure that rolling shutter projection would not be out of image */
    const Precision kBorderSize = 50;
    rng = std::mt19937(static_cast<unsigned int>(time(0)));
    Eigen::Vector2<Precision> focals = Eigen::Vector2<Precision>(448, 448);
    Eigen::Vector2<Precision> img_size = Eigen::Vector2<Precision>(1280, 720);
    pinhole_cam_ =
        std::make_unique<model::PinholeCamera<Precision>>(img_size, focals, Eigen::Vector2<Precision>(640, 320));
    Precision k1 = 0.022297781517582316_p, k2 = -0.02955120683821621_p;

    radial_cam_ = std::make_unique<model::SimpleRadialCamera<Precision>>(img_size, 509,
                                                                         Eigen::Vector2<Precision>(640, 360), k1, k2);

    pX = std::uniform_real_distribution<Precision>(kBorderSize, img_size(0) - 1 - kBorderSize);
    pY = std::uniform_real_distribution<Precision>(kBorderSize, img_size(1) - 1 - kBorderSize);
    angle = std::uniform_real_distribution<Precision>(-std::numbers::pi_v<Precision> / 30,
                                                      std::numbers::pi_v<Precision> / 30);

    srand(static_cast<unsigned int>(time(0)));
  }
  void TearDown() override {}

 protected:
  std::unique_ptr<model::PinholeCamera<Precision>> pinhole_cam_;
  std::unique_ptr<model::SimpleRadialCamera<Precision>> radial_cam_;

  const int kTestNumber_ = 10;
  std::mt19937 rng;
  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;

  std::uniform_real_distribution<Precision> angle;
};

struct AlmostPinholeCamera : public model::PinholeCamera<Precision> {
  AlmostPinholeCamera(const model::PinholeCamera<Precision> &cam) : model::PinholeCamera<Precision>(cam) {}
};

TEST_F(testEpipolarLine, PinHohohole) {
  using Model = model::PinholeCamera<Precision>;

  model::PinholeCamera model(Eigen::Vector2<Precision>(1280, 640), Eigen::Vector2<Precision>(448, 448),
                             Eigen::Vector2<Precision>(640, 320));
  AlmostPinholeCamera almost_model(model);

  for (int j = 0; j < kTestNumber_; ++j) {
    Eigen::Vector3<Precision> trans(static_cast<Precision>(rand() % 10 - 20), static_cast<Precision>(rand() % 10 - 20),
                                    static_cast<Precision>(rand() % 10 - 20));
    Sophus::SO3<Precision> q_SO3 = Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(0, angle(rng), 0));
    Sophus::SE3<Precision> pose_(q_SO3, trans);

    motion::SE3<Precision> pose(pose_);
    EpipolarLineBuilder<Model, motion::SE3<Precision>> builder(model, pose);
    EpipolarLineBuilder<AlmostPinholeCamera, motion::SE3<Precision>> almost_builder(almost_model, pose);

    reprojection::ArrayReprojector<Precision, Model, motion::SE3<Precision>> reprojector_init(model, pose);

    for (int i = 0; i < kTestNumber_; ++i) {
      Eigen::Vector2<Precision> input(pX(rng), pY(rng)), target;

      Precision idepth = 1 / 20;
      if (!reprojector_init.reproject(input, idepth, target)) {
        i -= 1;
        continue;
      }

      auto line = builder.build(input);
      auto almost_line = almost_builder.build(input);
      for (auto &almost_pt : almost_line.points) {
        Precision max_d = std::numeric_limits<Precision>::max();
        for (auto &pt : line.points) {
          max_d = std::min(max_d, (almost_pt.projection - pt.projection).norm());
        }
        EXPECT_LE(max_d, 1.5);
      }

#if DRAW_EPIPOLAR_LINES
      cv::Mat img = cv::Mat(int(model.image_size()(1)), int(model.image_size()(0)), CV_8UC3, cv::Scalar(0, 0, 0));
      for (auto &pt : line.points) {
        cv::circle(img, cv::Point(int(pt.projection.x()), int(pt.projection.y())), 3, cv::Scalar(0, 255, 0), -1);
      }

      for (auto &pt : almost_line.points) {
        cv::circle(img, cv::Point(int(pt.projection.x()), int(pt.projection.y())), 2, cv::Scalar(0, 0, 255), -1);
      }
      cv::imshow("Epipolar line", img);
      cv::waitKey(0);

#endif
    }
  }
}
