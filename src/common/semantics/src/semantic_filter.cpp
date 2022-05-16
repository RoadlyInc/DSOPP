#include "semantics/semantic_filter.hpp"

namespace dsopp::semantics {

SemanticFilter::SemanticFilter(const SemanticLegend &legend, const std::vector<std::string> &filter_names,
                               bool filter_indicated, bool filter_by_semantic)
    : filter_by_semantic_(filter_by_semantic) {
  for (const auto &tag : legend.tags()) {
    auto found = std::find(filter_names.begin(), filter_names.end(), tag.name) != filter_names.end();
    is_filtered_[tag.code] = filter_indicated ? found : !found;
  }
}

bool SemanticFilter::filtered(size_t code) const { return is_filtered_[code]; }

bool SemanticFilter::filterBySemantic() const { return filter_by_semantic_; }

}  // namespace dsopp::semantics
