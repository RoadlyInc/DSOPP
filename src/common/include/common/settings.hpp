#ifndef DSOPP_COMMON_SETTINGS_HPP
#define DSOPP_COMMON_SETTINGS_HPP

#include <cstddef>

#include <Eigen/Core>

namespace dsopp {
/** precision of all computation (float is faster but less accurate) */
#if USE_FLOAT
using Precision = float;
#else
using Precision = double;
#endif
constexpr Precision operator"" _p(long double arg) { return static_cast<Precision>(arg); }
constexpr Precision operator"" _p(unsigned long long arg) { return static_cast<Precision>(arg); }
/** Aligned allocator for the current Precision */
using PrecisionAllocator = Eigen::aligned_allocator<Precision>;
}  // namespace dsopp

#endif  // DSOPP_COMMON_SETTINGS_HPP
