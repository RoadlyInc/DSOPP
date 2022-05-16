#include "visualizer/visualizer_text_output_interface.hpp"
#include <mutex>

namespace dsopp::output {

VisualizerTextOutputInterface::VisualizerTextOutputInterface(const std::string &field_name)
    : text_field_name_(field_name) {}

void VisualizerTextOutputInterface::pushText(const std::string &text) {
  std::lock_guard lock(mutex_);
  update_text_ = true;
  text_to_show_ = text;
}

void VisualizerTextOutputInterface::render() {
  std::lock_guard lock(mutex_);
  if (update_text_) {
    *text_variable_ = text_to_show_;
    update_text_ = false;
  }
}

void VisualizerTextOutputInterface::init(pangolin::View &) {
  // NOTE that `META_FLAG_READONLY` here is needed, cuz otherwise field stops updating with mouse press
  text_variable_ =
      std::make_unique<pangolin::Var<std::string>>("status." + text_field_name_, "", pangolin::META_FLAG_READONLY);
}

VisualizerTextOutputInterface::~VisualizerTextOutputInterface() = default;

}  // namespace dsopp::output
