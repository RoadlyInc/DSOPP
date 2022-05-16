#include "common/image_tools/conversion.hpp"

#include <vector>

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp::common::image_tools {

template <int ChannelsNumber>
cv::Mat pixelMap2Mat1C(const features::PixelMap<ChannelsNumber> &pixel_map, int channel_index) {
  using MatD = Eigen::Matrix<dsopp::Precision, -1, -1, Eigen::RowMajor>;

  cv::Mat dst = cv::Mat(static_cast<int>(pixel_map.height()), static_cast<int>(pixel_map.width()),
                        std::is_same_v<dsopp::Precision, double> ? CV_64FC1 : CV_32FC1);

  Eigen::Map<MatD> output(&dst.at<dsopp::Precision>(0, 0), pixel_map.height(), pixel_map.width());
  Eigen::Map<const MatD> input(pixel_map.data().data() + channel_index * pixel_map.height() * pixel_map.width(),
                               pixel_map.height(), pixel_map.width());

  output = input;
  output = output.array() - output.minCoeff();
  output *= 255 / output.maxCoeff();

  dst.convertTo(dst, CV_8UC1);

  return dst;
}

template <int ChannelsNumber>
cv::Mat pixelMap2Mat3C(const features::PixelMap<ChannelsNumber> &pixel_map) {
  std::vector<cv::Mat> channels(3);
  for (int channel_index = 0; channel_index < 3; ++channel_index) {
    channels[static_cast<size_t>(channel_index)] = pixelMap2Mat1C(pixel_map, channel_index);
  }

  cv::Mat image;
  cv::merge(channels, image);

  return image;
}

template <int ChannelsNumber>
cv::Mat pixelMap2Mat3C(const features::PixelMap<ChannelsNumber> &pixel_map,
                       const Eigen::Matrix<Precision, 3, ChannelsNumber> &pca) {
  const int kRGB = 3;
  int height = static_cast<int>(pixel_map.height());
  int width = static_cast<int>(pixel_map.width());
  std::vector<cv::Mat> channels = {cv::Mat(height, width, CV_64F), cv::Mat(height, width, CV_64F),
                                   cv::Mat(height, width, CV_64F)};

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Eigen::Vector<Precision, ChannelsNumber> &pixel_in = pixel_map(x, y).intensity();
      Eigen::Vector<Precision, kRGB> pixel_out = pca * pixel_in;
      for (size_t channel_index = 0; channel_index < kRGB; ++channel_index) {
        channels[channel_index].at<Precision>(y, x) = pixel_out(static_cast<int>(channel_index));
      }
    }
  }

  for (size_t channel_index = 0; channel_index < kRGB; ++channel_index) {
    double min, max;
    minMaxIdx(channels[channel_index], &min, &max);
    double alpha = 255. / (max - min);
    double beta = -255. * min / (max - min);
    channels[channel_index].convertTo(channels[channel_index], CV_8UC1, alpha, beta);
  }

  cv::Mat image;
  cv::merge(channels, image);

  return image;
}

template cv::Mat pixelMap2Mat1C<1>(const features::PixelMap<1> &, int);
template cv::Mat pixelMap2Mat1C<3>(const features::PixelMap<3> &, int);
template cv::Mat pixelMap2Mat1C<8>(const features::PixelMap<8> &, int);
template cv::Mat pixelMap2Mat1C<16>(const features::PixelMap<16> &, int);
template cv::Mat pixelMap2Mat1C<32>(const features::PixelMap<32> &, int);
template cv::Mat pixelMap2Mat1C<128>(const features::PixelMap<128> &, int);

template cv::Mat pixelMap2Mat3C<32>(const features::PixelMap<32> &);
template cv::Mat pixelMap2Mat3C<128>(const features::PixelMap<128> &);

}  // namespace dsopp::common::image_tools
