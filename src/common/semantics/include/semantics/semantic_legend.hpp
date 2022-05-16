#ifndef DSOPP_COMMON_SEMANTICS_SEMANTIC_LEGEND_HPP_
#define DSOPP_COMMON_SEMANTICS_SEMANTIC_LEGEND_HPP_

#include <fstream>
#include <map>
#include <string>

#include <Eigen/Eigen>

#include "semantic_legend.pb.h"

namespace cv {
class Mat;
}
namespace dsopp {
namespace semantics {
/**
 * stores information on semantic class
 *
 */
struct SemanticTag {
  /** code of the tag from 0 to 255 */
  size_t code;
  /** name of the semantic class*/
  std::string name;
  /** Indicates the priority of a type over others. For example can be zero for background */
  size_t weight = 10;
};
/**
 * Legend for the semantic segmentation frame. Each intensity on the gray scale frame is assigned a semantic tag.
 */
class SemanticLegend {
 public:
  /** Maximum number of semantic types. */
  const static size_t kMaxNumberOfTypes = static_cast<size_t>(std::numeric_limits<uint8_t>::max()) + 1;
  /**
   * creates SemanticLegend from the file
   * @param file file to read legend
   */
  SemanticLegend(std::ifstream &file);
  /**
   * @param code id of the tag
   * @return weight of the tag with the given code
   */
  size_t weight(size_t code) const;
  /**
   * @return all tags from the legend
   */
  const std::vector<SemanticTag> &tags() const;
  /**
   * creates legend from the protobuf container
   * @param proto protobuf container
   */
  explicit SemanticLegend(const sensors::calibration::proto::SemanticLegend &proto);
  /**
   * creates protobuf container from the object
   * @return protobuf container
   */
  sensors::calibration::proto::SemanticLegend proto() const;

 private:
  /** all semantics types */
  std::vector<SemanticTag> semantic_tags_;
  /** quick check if the tags weight */
  std::array<size_t, kMaxNumberOfTypes> weights_ = {1};
};

}  // namespace semantics
}  // namespace dsopp

#endif  // DSOPP_COMMON_SEMANTICS_SEMANTIC_LEGEND_HPP_
