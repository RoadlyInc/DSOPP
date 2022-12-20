
#include "common/settings.hpp"
#include "features/camera/pixel_map.hpp"

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using namespace dsopp;
using namespace dsopp::features;

template <int channels>
void accelerated_dxdy_test_body() {
  constexpr int width = 512;
  constexpr int height = 512;

  // to make sure the optimized calculaton routine will be used
  static_assert(width % 8 == 0);
  static_assert(height % 8 == 0);

  // read an image
  const cv::Mat initial_image = cv::imread(TEST_DATA_DIR "rotation_pair/img1.png");
  cv::Mat image;
  cv::resize(initial_image, image, cv::Size(width, height));

  // fill the input buffer `channels` times
  std::vector<Precision, PrecisionAllocator> image_data;
  image_data.resize(static_cast<size_t>(width * height * channels));
  for (int c = 0; c < channels; c++)
    for (int i = 0; i < width * height; ++i)
      image_data[static_cast<size_t>(width * height * c + i)] = image.at<uint8_t>(i);

  // run the legacy calculation
  std::vector<features::PixelInfo<channels>, typename features::PixelInfo<channels>::PixelInfoAllocator>
      pixelinfo_eigen;
  pixelinfo_eigen.resize(static_cast<size_t>(width * height));
  {
    typename features::PixelMap<channels>::PixelInfoStorage map(pixelinfo_eigen.data(), height, width);

    using Channel = Eigen::Map<Eigen::Matrix<Precision, -1, -1, Eigen::RowMajor>>;
    for (int i = 0; i < channels; ++i) {
      Channel channel(&image_data[static_cast<size_t>(i * width * height)], height, width);

      for (long y = 0; y < height; ++y)
        for (long x = 0; x < width; ++x) {
          Precision c = channel(y, x);
          map(y, x).data_(i, 0) = c;

          Precision dx;
          if (x == 0) {
            dx = channel(y, x + 1) - channel(y, x);
          } else if (x == width - 1) {
            dx = channel(y, x) - channel(y, x - 1);
          } else {
            dx = 0.5_p * (channel(y, x + 1) - channel(y, x - 1));
          }
          map(y, x).data_(i, 1) = dx;

          Precision dy;
          if (y == 0) {
            dy = channel(y + 1, x) - channel(y, x);
          } else if (y == height - 1) {
            dy = channel(y, x) - channel(y - 1, x);
          } else {
            dy = 0.5_p * (channel(y + 1, x) - channel(y - 1, x));
          }
          map(y, x).data_(i, 2) = dy;
        }
    }
  }

  // run the optimized calculation
  PixelMap<channels> pixelmap_accelerated(std::move(image_data), width, height);

  // compare the results
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      auto v1 = pixelmap_accelerated(static_cast<size_t>(y * width + x));
      auto v2 = pixelinfo_eigen.at(static_cast<size_t>(y * width + x));
      EXPECT_EQ(v1.data_, v2.data_);
    }
}

TEST(PixelInfoTest, AcceleratedDxDyC1) { accelerated_dxdy_test_body<1>(); }

TEST(PixelInfoTest, AcceleratedDxDyC8) { accelerated_dxdy_test_body<8>(); }
