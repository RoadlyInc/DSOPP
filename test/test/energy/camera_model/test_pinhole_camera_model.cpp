
#include <memory>
#include <numbers>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/epipolar_geometry/epipolar_line.hpp"
#include "energy/epipolar_geometry/epipolar_line_builder.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "generic_camera_test.hpp"

namespace dsopp {
namespace energy {
namespace model {
class testPinholeCamera : public ::testing::Test {
 public:
  using Vec3 = Eigen::Matrix<Precision, 3, 1>;
  using Vec2 = Eigen::Matrix<Precision, 2, 1>;
  using SE3 = energy::motion::SE3<Precision>;

 protected:
  void SetUp() override {
    const Precision kBorderSize = PinholeCamera<Precision>::kBorderSize;
    focals = Eigen::Vector2<Precision>(448, 448);
    img_size = Eigen::Vector2<Precision>(1280, 640);
    cam_ = std::make_unique<PinholeCamera<Precision>>(img_size, focals, Eigen::Vector2<Precision>(640, 320));

    Precision kEps = 1e-3_p;
    pX = std::uniform_real_distribution<Precision>(kBorderSize + kEps, img_size(0) - 1 - kBorderSize - kEps);
    pY = std::uniform_real_distribution<Precision>(kBorderSize + kEps, img_size(1) - 1 - kBorderSize - kEps);
  }
  void TearDown() override {}
  std::unique_ptr<PinholeCamera<Precision>> cam_;

  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> img_size;

  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
};

TEST_F(testPinholeCamera, TestFocalLengths) { TestFocalsEqual(cam_.get(), focals); }
TEST_F(testPinholeCamera, ProjectUnprojectTest) { TestProjectUnproject(cam_.get(), pX, pY); }
TEST_F(testPinholeCamera, UnProjectprojectTest) { TestUnprojectProject(cam_.get(), pX, pY); }
TEST_F(testPinholeCamera, ProjectionJacobianTest) { TestProjectJacobian(cam_.get(), pX, pY); }

TEST_F(testPinholeCamera, pointEpipolarLine) {
  Vec2 observed_point(640, 320);
  SE3 g;
  g.translation() = Vec3(0, 0, 1);

  auto epiline_builder = energy::epipolar_geometry::EpipolarLineBuilder<PinholeCamera<Precision>, SE3>(*cam_, g);
  auto epipolar_line = epiline_builder.build(observed_point);

  for (const auto &point : epipolar_line.points) {
    ASSERT_TRUE((observed_point - point.projection).norm() < 1e-12);
    ASSERT_TRUE((observed_point - point.projection).norm() < 1e-12);
    ASSERT_EQ(point.reference_idepth, 0);
  }
}

TEST_F(testPinholeCamera, generalEpipolarLine) {
  int num_of_random_tests = 1000;
  const Precision kBorderSize = PinholeCamera<Precision>::kBorderSize;
  std::random_device random_device;
  std::mt19937 generator(random_device());

  for (int n = 0; n < num_of_random_tests; n++) {
    Vec2 observed_point(pX(generator), pY(generator));
    SE3 g;
    g.translation() = Vec3(static_cast<Precision>(rand() % 20 - 10), static_cast<Precision>(rand() % 20 - 10),
                           static_cast<Precision>(rand() % 20 - 10));
    std::mt19937 rng;
    std::uniform_real_distribution<Precision> angle(-std::numbers::pi_v<Precision> / 3,
                                                    std::numbers::pi_v<Precision> / 3);
    Sophus::SO3<Precision> q_so3 =
        Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(angle(rng), angle(rng), angle(rng)));
    g.setQuaternion(q_so3.unit_quaternion());
    energy::reprojection::ArrayReprojector<Precision, energy::model::PinholeCamera<Precision>,
                                           energy::motion::SE3<Precision>>
        reprojector(*cam_, g);

    auto epiline_builder = energy::epipolar_geometry::EpipolarLineBuilder<PinholeCamera<Precision>, SE3>(*cam_, g);
    auto epipolar_line = epiline_builder.build(observed_point).points;

    // check that if the epipolar_line == nullptr, no point is reprojected
    if (epipolar_line.empty()) {
      for (Precision i = 0; i < 1e6; i++) {
        Precision idepth = i / 1e3_p;
        Eigen::Vector2<Precision> x;
        ASSERT_FALSE(reprojector.reproject(observed_point, idepth, x)) << idepth << " : " << x.transpose();
      }
      continue;
    }
    // test that there is no empty epipolar lines
    size_t counter = 1;
    for (Precision i = 0; i < 1e6; i++) {
      Precision idepth = i / 1e3_p;
      Eigen::Vector2<Precision> x;
      if (reprojector.reproject(observed_point, idepth, x)) {
        counter++;
      }
    }
    ASSERT_TRUE(counter > 1 or
                std::abs(epipolar_line.front().reference_idepth - epipolar_line.back().reference_idepth) < 0.001);

    std::vector<Vec2> line;
    for (const auto &point : epipolar_line) {
      Eigen::Vector2<Precision> x;
      ASSERT_TRUE(reprojector.reproject(observed_point, point.reference_idepth, x) or
                  std::abs(x(0) - kBorderSize) < 1e-1 or std::abs(x(0) - img_size(0) + kBorderSize + 1) < 1e-1 or
                  std::abs(x(1) - kBorderSize) < 1e-1 or std::abs(x(1) - img_size(1) + kBorderSize + 1) < 1e-1)
          << point.reference_idepth << " : " << x.transpose();
      line.emplace_back(x);
    }
    // check that there is no gaps in the line
    for (size_t i = 0; i + 1 < line.size(); i++) {
      Precision gap = (line[i + 1] - line[i]).norm();
      ASSERT_TRUE(gap < 2) << gap;
    }
    // check that all idepth exists and has correct projections
    Precision max_id = std::max(epipolar_line.front().reference_idepth, epipolar_line.back().reference_idepth) + 1e-4_p;
    Precision min_id = std::min(epipolar_line.front().reference_idepth, epipolar_line.back().reference_idepth) - 1e-4_p;
    Precision start = min_id + (max_id - min_id) / 4;
    Precision end = min_id + 2 * (max_id - min_id) / 4;

    auto epipolar_line_s = epiline_builder.buildSegment(observed_point, start, end).points;

    if (epipolar_line_s.empty()) {
      for (const auto &point : epipolar_line_s) {
        Eigen::Vector2<Precision> x;
        ASSERT_TRUE(reprojector.reproject(observed_point, point.reference_idepth, x));
        ASSERT_TRUE((x - point.projection).norm() < 1e-13);
      }
    }
    for (Precision i = 0; i < 1e6; i++) {
      Precision idepth = i / 1e3_p;
      Eigen::Vector2<Precision> x;
      if (reprojector.reproject(observed_point, idepth, x)) {
        ASSERT_TRUE(idepth >= min_id - 1e-1 and idepth <= max_id + 1e1)
            << idepth << " not in [" << min_id << ", " << max_id << "]";
      }
    }
  }
}

TEST_F(testPinholeCamera, noEpipolarLine) {
  Vec2 observed_point(640, 320);
  SE3 g;
  auto epiline_builder = energy::epipolar_geometry::EpipolarLineBuilder<PinholeCamera<Precision>, SE3>(*cam_, g);
  auto epipolar_line = epiline_builder.build(observed_point).points;
  ASSERT_TRUE(epipolar_line.empty());
}

}  // namespace model
}  // namespace energy
}  // namespace dsopp
