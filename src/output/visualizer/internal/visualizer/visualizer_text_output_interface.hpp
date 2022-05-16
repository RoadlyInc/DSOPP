#ifndef DSOPP_OUTPUT_VISUALIZER_VISUALIZER_TEXT_OUTPUT_INTERFACE_HPP_
#define DSOPP_OUTPUT_VISUALIZER_VISUALIZER_TEXT_OUTPUT_INTERFACE_HPP_

#include <memory>
#include <string>

#include <pangolin/pangolin.h>
#include <pangolin/var/varvaluegeneric.h>
#include "output_interfaces/text_output_interface.hpp"
#include "visualizer/renderable.hpp"
#include "visualizer/visualizer_output_interface.hpp"

namespace dsopp::output {

/**
 * text output interface for visualization:
 * create a box in visualizer with the functionality of rendering text
 */
class VisualizerTextOutputInterface : public TextOutputInterface, public Renderable, public VisualizerOutputInterface {
 public:
  /**
   * creates output interface with `status.{field_name}` name in pangolin `status` window
   * @param field_name field name
   */
  VisualizerTextOutputInterface(const std::string &field_name);

  void pushText(const std::string &text) override;
  /**
   * sets device where the text would be rendered
   */
  void init(pangolin::View &) override;

  /**
   * Stores saved text into `pangolin::Var`
   */
  void render() override;
  ~VisualizerTextOutputInterface() override;

 private:
  /** pangolin text variable storage */
  std::unique_ptr<pangolin::Var<std::string>> text_variable_;
  /** output device where the text is rendered*/
  std::string text_field_name_;
  /** to push value into Var */
  std::mutex mutex_;
  /** cached new string */
  std::string text_to_show_;
  /** update text indicator */
  bool update_text_ = false;
};
}  // namespace dsopp::output

#endif  // DSOPP_OUTPUT_VISUALIZER_VISUALIZER_TEXT_OUTPUT_INTERFACE_HPP_
