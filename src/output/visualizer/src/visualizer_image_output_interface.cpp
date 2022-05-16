#include "visualizer/visualizer_image_output_interface.hpp"

#include <glog/logging.h>
#include <pangolin/pangolin.h>

namespace dsopp {
namespace output {
namespace {
const int kTextureResolutionUpscale = 2;
}
VisualizerImageOutputInterface::VisualizerImageOutputInterface(int width, int height, int position_x, int position_y)
    : width_(width), height_(height), position_x_(position_x), position_y_(position_y) {
  image_.resize(static_cast<size_t>(3 * width * height * kTextureResolutionUpscale * kTextureResolutionUpscale));
}

void VisualizerImageOutputInterface::pushImage(const cv::Mat image) {
  cv::Mat sprite;
  if (image.type() == CV_8UC1) {
    cv::cvtColor(image, sprite, cv::COLOR_GRAY2RGB);
  } else if (image.type() == CV_8UC3) {
    cv::cvtColor(image, sprite, cv::COLOR_BGR2RGB);
  } else {
    LOG(ERROR) << "can't push image, unsupported type";
  }
  cv::resize(sprite, sprite, cv::Size(height_ * kTextureResolutionUpscale, width_ * kTextureResolutionUpscale));
  cv::flip(sprite, sprite, 0);
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::memcpy(image_.data(), sprite.data, image_.size());
    reload_texture_ = true;
  }
}
int VisualizerImageOutputInterface::width() { return width_; }
int VisualizerImageOutputInterface::height() { return height_; }
void VisualizerImageOutputInterface::render() {
  if (reload_texture_) {
    const std::lock_guard<std::mutex> lock(mutex_);
    texture_.Upload(image_.data(), GL_RGB, GL_UNSIGNED_BYTE);
    reload_texture_ = false;
  }
  output_device_->Activate();
  glColor3f(1.0, 1.0, 1.0);
  texture_.RenderToViewport();
}

void VisualizerImageOutputInterface::init(pangolin::View &main_view) {
  pangolin::View *output_device = &pangolin::CreateDisplay().SetBounds(
      pangolin::Attach::Pix(position_x_), pangolin::Attach::Pix(position_x_ + height()),
      pangolin::Attach::Pix(position_y_), pangolin::Attach::Pix(position_y_ + width()));
  main_view.AddDisplay(*output_device);
  texture_ = pangolin::GlTexture(height_ * kTextureResolutionUpscale, width_ * kTextureResolutionUpscale, GL_RGB, false,
                                 0, GL_RGB, GL_UNSIGNED_BYTE);
  output_device_ = output_device;
}
VisualizerImageOutputInterface::~VisualizerImageOutputInterface() = default;
}  // namespace output
}  // namespace dsopp
