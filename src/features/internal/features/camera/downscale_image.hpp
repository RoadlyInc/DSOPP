#ifndef DSOPP_DOWNSCALE_IMAGE_HPP
#define DSOPP_DOWNSCALE_IMAGE_HPP

#include <Eigen/Dense>
#include <array>

namespace dsopp::features {
/**
 * fast resize image 2 times
 * @param image grayscale uchar image stored as row major
 * @param height, width size of the image
 */
static std::vector<Precision> downscaleImage(const std::vector<Precision> &image, int height, int width) {
  Eigen::Map<const Eigen::Matrix<Precision, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> image_map(image.data(),
                                                                                                        height, width);
  std::vector<Precision> resized_image(image.size() / 4);
  Eigen::Map<Eigen::Matrix<Precision, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> resized_image_map(
      resized_image.data(), height / 2, width / 2);

  resized_image_map = 0.25_p * (image_map(Eigen::seq(Eigen::fix<0>, height - 1, Eigen::fix<2>),
                                          Eigen::seq(Eigen::fix<0>, width - 1, Eigen::fix<2>)) +
                                image_map(Eigen::seq(Eigen::fix<1>, height, Eigen::fix<2>),
                                          Eigen::seq(Eigen::fix<1>, width, Eigen::fix<2>)) +
                                image_map(Eigen::seq(Eigen::fix<0>, height - 1, Eigen::fix<2>),
                                          Eigen::seq(Eigen::fix<1>, width, Eigen::fix<2>)) +
                                image_map(Eigen::seq(Eigen::fix<1>, height, Eigen::fix<2>),
                                          Eigen::seq(Eigen::fix<0>, width - 1, Eigen::fix<2>)));
  return resized_image;
}

}  // namespace dsopp::features

#endif  // DSOPP_DOWNSCALE_IMAGE_HPP
