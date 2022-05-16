#include "features/camera/gn_net_extractor.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "common/image_tools/conversion.hpp"
#include "common/settings.hpp"

#define DEBUG_IMSHOW COMPILE_HIDDEN_CODE

/**
 * Utility to run image through neural network
 * it will `imshow` first channel of output image
 */

template <int Channels>
void runImage(const std::string &model, const cv::Mat &image) {
  auto network = dsopp::features::GNNetExtractor<Channels>(model);

  auto time_start = std::chrono::system_clock::now();

  int kNumberOfTests = 50;

  auto ans = network.extract(image);
  for (int i = 1; i < kNumberOfTests; ++i) {
    ans = network.extract(image);
  }
  auto time_stop = std::chrono::system_clock::now();

  std::cout << "Average iteration time : "
            << (std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start).count() / kNumberOfTests)
            << " ms" << std::endl;

#if DEBUG_IMSHOW
  cv::Mat dst = dsopp::common::image_tools::pixelMap2Mat1C(*ans);

  cv::imshow(model, dst);
  cv::waitKey(0);

#endif
}

template <int Channels>
void runPyramids(const std::string &model, const cv::Mat &image) {
  auto network = dsopp::features::GNNetExtractor<Channels>(model);

  auto time_start = std::chrono::system_clock::now();

  int kNumberOfTests = 50;

  auto ans = network.extractPyramids(image);
  for (int i = 1; i < kNumberOfTests; ++i) {
    ans = network.extractPyramids(image);
  }
  auto time_stop = std::chrono::system_clock::now();

  std::cout << "Average iteration time : "
            << (std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start).count() / kNumberOfTests)
            << " ms" << std::endl;

#if DEBUG_IMSHOW
  for (size_t i = 0; i < ans.size(); ++i) {
    cv::Mat dst = dsopp::common::image_tools::pixelMap2Mat1C(ans[i]);

    cv::imshow(model + std::to_string(i), dst);
  }

  cv::waitKey(0);
#endif
}

int main(int, char **) {
  std::string jit_model_path_8 = TEST_DATA_DIR "8channel.model.pt";

  cv::Mat initial_image = cv::imread(TEST_DATA_DIR "rotation_pair/img1.png");

  std::cout << "Running 8-channel light model ``image`` extraction: \n";
  runImage<8>(jit_model_path_8, initial_image);
  std::cout << "Running 8-channel light model ``pyramid`` extraction: \n";
  runPyramids<8>(jit_model_path_8, initial_image);

  return 0;
}
