
#include <array>
#include <memory>
#include <string>

#include <benchmark/benchmark.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_data_frame_extractor.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

using namespace dsopp;
using namespace dsopp::features;
using namespace dsopp::sensors::calibration;

template <int width = 1280, int height = 720>
struct BenchmarkData {
  BenchmarkData() {
    const size_t kNumberOfPyramidLevels = 4;
    std::string image_name = TEST_DATA_DIR "lossless/00001.png";
    image = cv::imread(image_name);
    cv::resize(image, image, cv::Size(width, height));

    image_data.resize(width * height);
    for (int i = 0; i < width * height; ++i) image_data[static_cast<size_t>(i)] = image.at<uint8_t>(i);

    vignetting = cv::Mat(image.rows, image.cols, CV_8UC1, 255);

    for (size_t i = 0; i < 256; ++i) {
      photometric_calibration[i] = static_cast<Precision>(i);
    }
    pixel_data_frame_extractor = std::make_unique<PixelDataFrameExtractor>(CameraCalibration::kNumberOfPyramidLevels);

    CameraMask camera_mask(image.rows, image.cols);
    pyramid_of_masks = std::make_unique<std::vector<CameraMask>>();
    for (size_t lvl = 0; lvl < kNumberOfPyramidLevels; lvl++) {
      auto resized_mask = camera_mask.resize(1._p / static_cast<Precision>(1 << lvl));
      pyramid_of_masks->push_back(resized_mask.data());
    }
    features = std::make_unique<CameraFeatures>(0, cv::Mat(image), std::move(image), dsopp::time(), *pyramid_of_masks,
                                                tracking_feature_extractor, *pixel_data_frame_extractor, nullptr);
  }

  std::unique_ptr<CameraFeatures> features;

  std::vector<Precision> image_data;

 private:
  std::unique_ptr<std::vector<CameraMask>> pyramid_of_masks;
  cv::Mat image;
  cv::Mat vignetting;
  std::array<Precision, 256> photometric_calibration;

  SobelTrackingFeaturesExtractor tracking_feature_extractor;
  std::unique_ptr<PixelDataFrameExtractor> pixel_data_frame_extractor;
};

template <int width, int height>
static void TrackingFeatureBenchmark(benchmark::State& state) {
  BenchmarkData<width, height> data;
  data.features->pixelData();
  for (auto _ : state) {
    data.features->moveTracking();
  }
}

template <int width, int height>
static void PixelDataFrameBenchmark(benchmark::State& state) {
  BenchmarkData<width, height> data;
  for (auto _ : state) {
    data.features->movePixelData();
  }
}
BENCHMARK_TEMPLATE(PixelDataFrameBenchmark, 1280, 720)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(PixelDataFrameBenchmark, 1920, 1080)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(PixelDataFrameBenchmark, 3840, 2160)->Unit(benchmark::kMillisecond);

template <int width, int height>
static void PixelMapBenchmark(benchmark::State& state) {
  BenchmarkData<width, height> data;
  for (auto _ : state) {
    state.PauseTiming();
    std::vector<Precision> image = data.image_data;
    state.ResumeTiming();

    PixelMap<1> pixel_map(std::move(image), width, height);
  }
}
BENCHMARK_TEMPLATE(PixelMapBenchmark, 1280, 720)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(PixelMapBenchmark, 1920, 1080)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(PixelMapBenchmark, 3840, 2160)->Unit(benchmark::kMillisecond);

static void PixelDataFrameEvaluateBenchmark(benchmark::State& state) {
  BenchmarkData data;
  auto frame = data.features->movePixelData();
  const int PatternSize = 8;
  Eigen::Matrix<Precision, PatternSize, 1> target_patch;
  Eigen::Matrix<Precision, 2, PatternSize> target_pattern =
      Eigen::Matrix<Precision, 2, PatternSize>::Constant(500) + Eigen::Matrix<Precision, 2, PatternSize>::Random() * 10;
  Eigen::Vector<Precision, PatternSize> d_intensity_u_diag;
  Eigen::Vector<Precision, PatternSize> d_intensity_v_diag;
  auto& grid = frame->pyramids()[0];
  for (auto _ : state) {
    grid.Evaluate(target_pattern, target_patch, d_intensity_u_diag, d_intensity_v_diag);
    benchmark::DoNotOptimize(target_patch);
    benchmark::DoNotOptimize(d_intensity_u_diag);
    benchmark::DoNotOptimize(d_intensity_v_diag);
  }
}
BENCHMARK(PixelDataFrameEvaluateBenchmark);

BENCHMARK_TEMPLATE(TrackingFeatureBenchmark, 1280, 720)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(TrackingFeatureBenchmark, 1920, 1080)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(TrackingFeatureBenchmark, 3840, 2160)->Unit(benchmark::kMillisecond);
