#ifndef DSOPP_MATRIX_ACCUMULATOR_HPP
#define DSOPP_MATRIX_ACCUMULATOR_HPP

#include <Eigen/Dense>

namespace dsopp {
namespace energy {
namespace problem {
/**
 * class for accumulation sum of matrices assuming they have the same order of magnitude
 *
 * https://en.wikipedia.org/wiki/Kahan_summation_algorithm
 * @tparam Scalar,Rows,Cols parameters of the matrix
 */
template <typename Scalar, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
class MatrixAccumulator {
 public:
  /**
   * @param rows,cols size of the matrix
   */
  MatrixAccumulator(long rows, long cols) {
    sum_ = Eigen::Matrix<Scalar, Rows, Cols>(rows, cols);
    sum_.setZero();

    compensation_ = Eigen::Matrix<Scalar, Rows, Cols>(rows, cols);
    compensation_.setZero();

    t_ = Eigen::Matrix<Scalar, Rows, Cols>(rows, cols);
    y_ = Eigen::Matrix<Scalar, Rows, Cols>(rows, cols);
  }
  MatrixAccumulator() {
    sum_.setZero();
    compensation_.setZero();
  }
  /**
   * adds summand to the system
   * @param summand summand to add
   */
  template <typename EigenDerived>
  void operator+=(const Eigen::MatrixBase<EigenDerived> &summand) {
    y_ = summand - compensation_;
    t_ = sum_ + y_;
    compensation_.noalias() = (t_ - sum_) - y_;
    sum_ = t_;
  }
  /**
   * @return accumulated value
   */
  Eigen::Matrix<Scalar, Rows, Cols> value() { return sum_; }

 private:
  /** internal variable for Kahan algorithm, stored here to prevent dynamic memmory reallocation for dynamic matricies
   */
  Eigen::Matrix<Scalar, Rows, Cols> y_;
  /** internal variable for Kahan algorithm, stored here to prevent dynamic memmory reallocation for dynamic matricies
   */
  Eigen::Matrix<Scalar, Rows, Cols> t_;
  /** running sum */
  Eigen::Matrix<Scalar, Rows, Cols> sum_;
  /** compensation for lower order digits */
  Eigen::Matrix<Scalar, Rows, Cols> compensation_;
};
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif
