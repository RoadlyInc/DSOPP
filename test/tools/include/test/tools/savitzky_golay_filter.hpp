#ifndef DSOPP_SAVITZKY_GOLAY_FILTER_HPP
#define DSOPP_SAVITZKY_GOLAY_FILTER_HPP

#include <vector>
#include "energy/motion/motion.hpp"

namespace dsopp {
namespace test_tools {
/**
 * Savitzky-Golay filter  fits polynomial of a given degree to set of points from sliding window to obtain new point
 * value. See paper
 * Savitzky, A.; Golay, M.J.E. (1964). "Smoothing and Differentiation of Data by Simplified Least Squares Procedures".
 * Analytical Chemistry. 36 (8): 1627–39.
 * for details
 * @param data_input input values (1D vector)
 * @param half_windows_size  half of sliding window size minus one (sliding window size is odd)
 * @param degree polynomial degree to fit
 * @return vector of filtered values
 */
template <energy::motion::Motion Motion>
std::vector<typename Motion::Scalar> SavitzkyGolayFilter(const std::vector<typename Motion::Scalar> &data_input,
                                                         size_t half_windows_size, size_t degree);

}  // namespace test_tools
}  // namespace dsopp
#endif  // DSOPP_SAVITZKY_GOLAY_FILTER_HPP
