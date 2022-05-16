#ifndef DSOPP_COMMON_SEMANTICS_SEMANTIC_FILTER_HPP_
#define DSOPP_COMMON_SEMANTICS_SEMANTIC_FILTER_HPP_

#include "semantics/semantic_legend.hpp"

namespace dsopp {
namespace semantics {

/**
 * Filter that use semantic data to include/exclude objects
 */
class SemanticFilter {
 public:
  /**
   * creates SemanticFilter from the legend
   * @param legend semantic legend
   * @param filter_names symantic types, which should be filtered
   * @param filter_indicated ``true`` to filter indicated names ``false`` to filter complement
   * @param filter_by_semantic use semantic information to filter objects
   */
  SemanticFilter(const SemanticLegend &legend, const std::vector<std::string> &filter_names, bool filter_indicated,
                 bool filter_by_semantic);
  /**
   * @return true if semantic information used to filter objects
   */
  bool filterBySemantic() const;
  /**
   * @param code id of the tag
   * @return true if the semantic type corresponding to the given code is dynamic and false otherwise
   */
  bool filtered(size_t code) const;

 private:
  /** quick check if the tag is filterable  */
  std::array<bool, SemanticLegend::kMaxNumberOfTypes> is_filtered_ = {false};
  /** use semantic information to filter objects */
  bool filter_by_semantic_;
};

}  // namespace semantics
}  // namespace dsopp

#endif  // DSOPP_COMMON_SEMANTICS_SEMANTIC_FILTER_HPP_
