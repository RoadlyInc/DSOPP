#ifndef DSOPP_OUTPUT_TEXT_OUTPUT_INTERFACE_HPP_
#define DSOPP_OUTPUT_TEXT_OUTPUT_INTERFACE_HPP_

#include <string>

namespace dsopp::output {

/**
 * \brief Output interface for text.
 *
 * An interface to push text info.
 */
class TextOutputInterface {
 public:
  /**
   * push text to be rendered
   * @param text input text
   */
  virtual void pushText(const std::string &text) = 0;
  virtual ~TextOutputInterface() = default;
};

}  // namespace dsopp::output

#endif  // DSOPP_OUTPUT_TEXT_OUTPUT_INTERFACE_HPP_
