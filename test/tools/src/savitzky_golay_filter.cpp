#include "test/tools/savitzky_golay_filter.hpp"

#include <cmath>
#include <cstddef>

#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace test_tools {
namespace {
/** calculate Savitzky-Golay coefficients. */
template <energy::motion::Motion Motion>
Eigen::VectorX<typename Motion::Scalar> SavitzkyGolayCoefficients(const Eigen::VectorX<typename Motion::Scalar> &b,
                                                                  size_t degree) {
  using Scalar = typename Motion::Scalar;
  const long rows(b.size());
  const long cols(static_cast<long>(degree + 1));

  Eigen::Matrix<Scalar, -1, -1> A;
  Eigen::VectorX<Scalar> result;
  A.resize(rows, cols);
  result.resize(rows);

  for (long i = 0; i < rows; ++i) {
    for (long j = 0; j < cols; ++j) {
      A(i, j) = std::pow(Scalar(i), Scalar(j));
    }
  }

  auto AtA = A.transpose() * A;
  auto Atb = A.transpose() * b;
  Eigen::Matrix<Scalar, -1, -1> c = AtA.ldlt().solve(Atb);

  for (long i = 0; i < static_cast<long>(b.size()); ++i) {
    result(i) = c(0, 0);
    for (long j = 1; j <= static_cast<long>(degree); ++j) {
      result(i) += c(j, 0) * std::pow(Scalar(i), Scalar(j));
    }
  }
  return result;
}
template Eigen::VectorX<Precision> SavitzkyGolayCoefficients<energy::motion::SE3<Precision>>(
    const Eigen::VectorX<Precision> &b, size_t degree);
}  // namespace

/**
 * \brief Savitzky Golay smoothing.
 *
 * This method means fitting a polynomial of degree 'degree' to a sliding window
 * of width 2*half_windows_size+1 throughout the data.  The needed coefficients are
 * generated dynamically by doing a least squares fit on a "symmetric" unit
 * vector of size 2*half_windows_size+1, e.g. for half_windows_size=2 b=(0,0,1,0,0).
 * evaluating the polynomial yields the sg-coefficients.  At the border non symmetric
 * vectors b are used.
 **/
template <energy::motion::Motion Motion>
std::vector<typename Motion::Scalar> SavitzkyGolayFilter(const std::vector<typename Motion::Scalar> &data_input,
                                                         size_t half_windows_size, size_t degree) {
  using Scalar = typename Motion::Scalar;
  using Vector = typename Eigen::VectorX<Scalar>;

  long data_size = static_cast<long>(data_input.size());
  std::vector<Scalar> output(static_cast<size_t>(data_size), 0.0);
  Eigen::Map<Vector> data_filtered(output.data(), data_size);

  if (half_windows_size < 1 || data_size < static_cast<long>(2 * half_windows_size + 2)) {
    return data_input;
  }
  long window_size = static_cast<long>(2 * half_windows_size + 1);
  long index_end = data_size - 1;
  // do a regular sliding window average
  if (degree == 0) {
    // handle border cases first because we need different coefficients
    for (long i = 0; i < static_cast<long>(half_windows_size); ++i) {
      const Scalar scale = Scalar(1.0) / Scalar(i + 1);
      Vector c1(half_windows_size);
      c1.setConstant(scale);

      for (long j = 0; j <= i; ++j) {
        data_filtered(i) += c1(j) * data_input[static_cast<size_t>(j)];
        data_filtered(index_end - i) += c1(j) * data_input[static_cast<size_t>(index_end - j)];
      }
    }
    // now loop over rest of data. reusing the "symmetric" coefficients.
    const Scalar scale = Scalar(1.0) / Scalar(window_size);
    Vector c2(window_size);
    c2.setConstant(scale);

    for (long i = 0; i <= (data_size - window_size); ++i) {
      for (long j = 0; j < window_size; ++j) {
        data_filtered(i + static_cast<long>(half_windows_size)) += c2(j) * data_input[static_cast<size_t>(i + j)];
      }
    }
    return output;
  }
  // handle border cases first because we need different coefficients
  for (long i = 0; i < static_cast<long>(half_windows_size); ++i) {
    Vector b1(window_size);
    b1.setZero();
    b1(i) = 1.0;

    Vector c1 = SavitzkyGolayCoefficients<Motion>(b1, static_cast<size_t>(degree));
    for (long j = 0; j < window_size; ++j) {
      data_filtered(i) += c1(j) * data_input[static_cast<size_t>(j)];
      data_filtered(index_end - i) += c1(j) * data_input[static_cast<size_t>(index_end - j)];
    }
  }
  // now loop over rest of data. reusing the "symmetric" coefficients.
  Vector b2(window_size);
  b2.setZero();
  b2(static_cast<long>(half_windows_size)) = 1.0;
  Vector c2 = SavitzkyGolayCoefficients<Motion>(b2, static_cast<size_t>(degree));

  for (long i = 0; i <= (data_size - window_size); ++i) {
    for (long j = 0; j < window_size; ++j) {
      data_filtered(i + static_cast<long>(half_windows_size)) += c2(j) * data_input[static_cast<size_t>(i + j)];
    }
  }
  return output;
}
template std::vector<Precision> SavitzkyGolayFilter<energy::motion::SE3<Precision>>(
    const std::vector<Precision> &data_input, size_t half_windows_size, size_t degree);
}  // namespace test_tools
}  // namespace dsopp