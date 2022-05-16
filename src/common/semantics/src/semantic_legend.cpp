#include "semantics/semantic_legend.hpp"

#include <glog/logging.h>

#include "common/file_tools/parsing.hpp"

namespace dsopp::semantics {

SemanticLegend::SemanticLegend(std::ifstream &file) {
  std::string line;
  while (std::getline(file, line)) {
    auto tokens = common::file_tools::splitLine(line, ' ');
    CHECK(tokens.size() == 2 || tokens.size() == 3) << "Invalid semantic legend format";
    size_t code = std::stoul(tokens[0]);
    std::string name = tokens[1];

    if (tokens.size() == 3) {
      size_t weight = std::stoul(tokens[2]);
      semantic_tags_.push_back(SemanticTag{code, name, weight});
    } else {
      semantic_tags_.push_back(SemanticTag{code, name});
    }

    weights_[code] = semantic_tags_.back().weight;
  }
}

size_t SemanticLegend::weight(size_t code) const { return weights_[code]; }

const std::vector<SemanticTag> &SemanticLegend::tags() const { return semantic_tags_; }

SemanticLegend::SemanticLegend(const sensors::calibration::proto::SemanticLegend &proto) {
  for (const auto &tag : proto.tags()) {
    weights_[tag.code()] = tag.weight();
    semantic_tags_.push_back(SemanticTag{tag.code(), tag.name(), tag.weight()});
  }
}

sensors::calibration::proto::SemanticLegend SemanticLegend::proto() const {
  sensors::calibration::proto::SemanticLegend legend;
  for (const auto &tag : semantic_tags_) {
    sensors::calibration::proto::SemanticTag proto_tag;
    (*proto_tag.mutable_name()) = tag.name;
    proto_tag.set_code(tag.code);
    proto_tag.set_weight(tag.weight);

    *legend.add_tags() = proto_tag;
  }
  return legend;
}

}  // namespace dsopp::semantics
