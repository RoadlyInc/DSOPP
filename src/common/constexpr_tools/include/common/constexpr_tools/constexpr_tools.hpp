#ifndef DSOPP_COMMON_CONSTEXPR_HPP
#define DSOPP_COMMON_CONSTEXPR_HPP

namespace dsopp::common::constexpr_tools {
template <int Start, int End, int Inc, class F>
constexpr void constexpr_for(F&& f) {
  if constexpr (Start != End) {
    f.template operator()<Start>();
    constexpr_for<Start + Inc, End, Inc>(f);
  }
}
}  // namespace dsopp::common::constexpr_tools

#endif  // DSOPP_COMMON_CONSTEXPR_HPP
