#ifndef DSOPP_VISUALIZER_IMAGE_OUTPUT_INTERFACE_HPP
#define DSOPP_VISUALIZER_IMAGE_OUTPUT_INTERFACE_HPP
#include <pangolin/pangolin.h>

#include "output_interfaces/image_output_interface.hpp"
#include "visualizer/renderable.hpp"
#include "visualizer/visualizer_output_interface.hpp"

namespace dsopp {
namespace output {
/**
 * image output interface for visualization:
 * create a box in the visualizer with the functionality of uploading a new image into it
 */
class VisualizerImageOutputInterface : public ImageOutputInterface,
                                       public Renderable,
                                       public VisualizerOutputInterface {
 public:
  /**
   * creates output interface of a given size
   * @param width,height size of the image
   * @param position_x,position_y position of the image
   */
  VisualizerImageOutputInterface(int width, int height, int position_x, int position_y);
  void pushImage(const cv::Mat image) override;
  void init(pangolin::View &main_view) override;
  /**
   * renders image to device
   */
  void render() override;
  /**
   * @return width of the device
   */
  int width();
  /**
   * @return height of the device
   */
  int height();
  ~VisualizerImageOutputInterface() override;

 private:
  /** width of the device */
  int width_;
  /** height of the device */
  int height_;
  /** position of the left bottom corner(x)*/
  int position_x_;
  /** position of the left bottom corner(y)*/
  int position_y_;
  /** texture holding the current image*/
  pangolin::GlTexture texture_;
  /** output device where the image is uploaded*/
  pangolin::View *output_device_;
  /** current image*/
  std::vector<unsigned char> image_;
  /** true if image is changed and have to be reloaded to texture*/
  bool reload_texture_ = true;
  /** mutex to lock storage while it changed */
  std::mutex mutex_;
};
}  // namespace output
}  // namespace dsopp
#endif  // DSOPP_VISUALIZER_IMAGE_OUTPUT_INTERFACE_HPP
