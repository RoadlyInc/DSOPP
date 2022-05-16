#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"

#include "energy/projector/camera_reproject.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

using namespace dsopp;
using Calibration = sensors::calibration::CameraCalibration;
using Model = energy::model::PinholeCamera<Precision>;
const int N = 8;

struct BenchmarkData {
  BenchmarkData()
      : t_t_r(Sophus::SE3<Precision>::exp(Sophus::SE3<Precision>::Tangent::Random() * 0.5)),
        calibration{Eigen::Vector2<Precision>(1280, 720), Eigen::Vector4<Precision>(448.15_p, 448.15_p, 640, 360),
                    energy::model::ModelType::kPinholeCamera},
        model(calibration.cameraModel<Model>()),
        reprojector{*model, t_t_r} {
    reference_pattern.colwise() += Eigen::Vector2<Precision>(600, 600);
  }

  Sophus::SE3<Precision> t_t_r;
  Calibration calibration;
  std::unique_ptr<Model> model;
  energy::reprojection::ArrayReprojector<Precision, Model, energy::motion::SE3<Precision>> reprojector;

  Eigen::Matrix<Precision, 2, N> reference_pattern = Eigen::Matrix<Precision, 2, N>::Random() * 3;
  Precision idepth = 0.3_p;
  Eigen::Matrix<Precision, 2, N> target_pattern;
  Eigen::Vector<Precision, N> d_u_idepth;
  Eigen::Vector<Precision, N> d_v_idepth;
  Eigen::Matrix<Precision, N, Sophus::SE3<Precision>::DoF> d_u_tReferenceTarget;
  Eigen::Matrix<Precision, N, Sophus::SE3<Precision>::DoF> d_v_tReferenceTarget;
};

static void ReprojectorBenchmarkJacobians(benchmark::State& state) {
  BenchmarkData data;
  for (auto _ : state) {
    data.reprojector.reprojectPattern(data.reference_pattern, data.idepth, data.target_pattern, data.d_u_idepth,
                                      data.d_v_idepth, data.d_u_tReferenceTarget, data.d_v_tReferenceTarget);
  }
}

static void ReprojectorBenchmark(benchmark::State& state) {
  BenchmarkData data;
  for (auto _ : state) {
    data.reprojector.reprojectPattern(data.reference_pattern, data.idepth, data.target_pattern);
  }
}

BENCHMARK(ReprojectorBenchmarkJacobians);
BENCHMARK(ReprojectorBenchmark);