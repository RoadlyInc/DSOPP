#include "features/camera/extraction_tools.hpp"

#include <vector>

#include <glog/logging.h>
#include <Eigen/Dense>

namespace dsopp::features {

std::vector<c10::IValue> runExtractor(const cv::Mat &img_initial, torch::jit::script::Module &model) {
  using MatF = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>;

  cv::Mat img = img_initial.clone();
  if (img.type() == CV_8UC3) {
    img.convertTo(img, CV_32FC3);
  } else if (img.type() != CV_32FC3) {
    LOG(ERROR) << "Unsupported cv::Mat format. Use CV_8UC3 or CV_32FC3 instead";
  }

  std::vector<float> data;
  data.resize(static_cast<size_t>(3 * img.cols * img.rows));

  const size_t kNumberOfChannels = 3;
  cv::Mat bgr[kNumberOfChannels];

  cv::split(img, bgr);
  size_t outer_stride = static_cast<size_t>(img.rows * img.cols);

  for (size_t c = 0; c < 3; ++c) {
    Eigen::Map<MatF> input(&bgr[c].at<float>(0, 0), img.rows, img.cols);
    Eigen::Map<MatF> output(&data[c * outer_stride], img.rows, img.cols);

    output = input;
  }

  long height = img_initial.rows;
  long width = img_initial.cols;

  /** disabling gradient evaluation */
  torch::NoGradGuard no_grad;

  torch::Tensor input = torch::from_blob(&data[0], {1, kNumberOfChannels, height, width});
  std::vector<torch::jit::IValue> inputs = {input};

  return model.forward(inputs).toTuple()->elements();
}

std::vector<Precision> flatten(torch::Tensor &model_output, int channels_number) {
  // Note that first dimension is batch dimension
  long output_channels = model_output.size(1);
  long output_height = model_output.size(2);
  long output_width = model_output.size(3);

  CHECK_EQ(channels_number, output_channels) << "Configure channel size and model channel sizes are different";

  std::vector<Precision> output_data(static_cast<size_t>(output_channels * output_height * output_width));

  auto model_output_c = model_output.contiguous();
  float *output_ptr = static_cast<float *>(model_output_c.data_ptr());

  long outer_stride = output_height * output_width;

  using MatF = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>;
  using MatD = Eigen::Matrix<Precision, -1, -1, Eigen::RowMajor>;

  for (long c = 0; c < output_channels; ++c) {
    size_t start_c = static_cast<size_t>(c * outer_stride);

    Eigen::Map<MatF> channel(&output_ptr[start_c], output_height, output_width);
    Eigen::Map<MatD> output_channel(&output_data[start_c], output_height, output_width);

    output_channel = channel.cast<Precision>();
  }
  return output_data;
}

}  // namespace dsopp::features
