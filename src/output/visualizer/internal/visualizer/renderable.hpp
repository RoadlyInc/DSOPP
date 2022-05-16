#ifndef DSOPP_RENDERABLE_HPP
#define DSOPP_RENDERABLE_HPP
namespace pangolin {
class View;
}
namespace dsopp {
namespace output {
struct OutputParameters;
/**
 * \brief Object of this class can render 3D scene.
 *
 * Object of this class can draw some objects in the 3D scene.
 */
class Renderable {
 public:
  /**
   * function to call on visualizer initialization
   */
  virtual void init(pangolin::View &){};
  /**
   * render objects in the 3D scene.
   */
  virtual void render() = 0;
  /**
   * set new parameters of rendering.
   * @param output_parameters new parameters
   */
  virtual void setParameters(const OutputParameters &output_parameters) { (void)output_parameters; };
  virtual ~Renderable() = default;
};

}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_RENDERABLE_HPP
