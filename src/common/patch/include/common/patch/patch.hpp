#ifndef DSOPP_COMMON_PATCH_HPP
#define DSOPP_COMMON_PATCH_HPP

#include <Eigen/Dense>

namespace dsopp {
namespace patches {
/** Patch */
template <int C>
struct Patch {
  /** patch storage order */
  static constexpr Eigen::StorageOptions StorageOrder = C == 1 ? Eigen::ColMajor : Eigen::RowMajor;
};
}  // namespace patches
/** alias for main patch storage order in all slam library */
template <int C>
constexpr Eigen::StorageOptions PatchStorageOrder = patches::Patch<C>::StorageOrder;
}  // namespace dsopp

#endif  // DSOPP_COMMON_PATCH_HPP
