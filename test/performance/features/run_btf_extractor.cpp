#include "features/camera/btf_extractor.hpp"

#include <iostream>
#include <string>
#include <vector>

#include <benchmark/benchmark.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "common/image_tools/conversion.hpp"
#include "common/settings.hpp"

#define DEBUG_IMSHOW COMPILE_HIDDEN_CODE

struct BenchmarkData {
 public:
  const std::string model = TEST_DATA_DIR "btf.model.pt";
  const cv::Mat image = cv::imread(TEST_DATA_DIR "rotation_pair/img1.png");
};

static void runImageBenchmark(benchmark::State& state) {
  BenchmarkData data;
  auto network = dsopp::features::BTFExtractor(data.model);

  for (auto _ : state) {
    benchmark::DoNotOptimize(network.extract(data.image));
  }

#if DEBUG_IMSHOW
  auto ans = network.extract(data.image);
  cv::Mat dst = dsopp::common::image_tools::pixelMap2Mat1C(*ans);
  cv::imshow(data.model, dst);
  cv::waitKey(0);

#endif
}

static void runPyramidsBenchmark(benchmark::State& state) {
  BenchmarkData data;
  auto network = dsopp::features::BTFExtractor(data.model);

  for (auto _ : state) {
    benchmark::DoNotOptimize(network.extractPyramids(data.image));
  }

#if DEBUG_IMSHOW
  auto ans = network.extractPyramids(data.image);

  auto dst0 = dsopp::common::image_tools::pixelMap2Mat1C(std::get<0>(*ans));
  cv::imshow(data.model + std::to_string(0), dst0);
  auto dst1 = dsopp::common::image_tools::pixelMap2Mat1C(std::get<1>(*ans));
  cv::imshow(data.model + std::to_string(1), dst1);
  auto dst2 = dsopp::common::image_tools::pixelMap2Mat1C(std::get<2>(*ans));
  cv::imshow(data.model + std::to_string(2), dst2);

  cv::waitKey(0);

#endif
}

BENCHMARK(runImageBenchmark)->Unit(benchmark::kMillisecond);
BENCHMARK(runPyramidsBenchmark)->Unit(benchmark::kMillisecond);
