#ifndef DSOPP_SRC_ENERGY_LINEAR_SYSTEM
#define DSOPP_SRC_ENERGY_LINEAR_SYSTEM

#include <glog/logging.h>
#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp {
namespace energy {
/** structure to store a normal equation of a form H*x=b
 *
 * it represents an equation obtain by a construction of a Gauss - Newton system
 */
template <typename Scalar = Precision, int SIZE = Eigen::Dynamic>
struct NormalLinearSystem {
  /**
   * creates system of a given size
   * @param size size of a system
   */
  NormalLinearSystem(int size = 0) {
    if constexpr (SIZE == Eigen::Dynamic) {
      H = Eigen::Matrix<Scalar, SIZE, SIZE>(size, size);
      b = Eigen::Vector<Scalar, SIZE>(size);
    }
  }
  /**
   * sets system zero
   */
  void setZero() {
    H.setZero();
    b.setZero();
  }
  /**
   * casts all element of equation to new type
   * @tparam T new type
   * @return casted system
   */
  template <typename T>
  typename std::conditional<std::is_same_v<T, Scalar>, const NormalLinearSystem<T, SIZE> &,
                            NormalLinearSystem<T, SIZE>>::type
  cast() const {
    if constexpr (std::is_same_v<T, Scalar>) {
      return *this;
    } else {
      NormalLinearSystem<T, SIZE> casted;
      casted.H = H.template cast<T>();
      casted.b = b.template cast<T>();
      return casted;
    }
  }

  /**
   * resizes system keeping previous elements
   * @param size new size;
   */
  void resize(long size) {
    if constexpr (SIZE == Eigen::Dynamic) {
      H.conservativeResize(size, size);
      b.conservativeResize(size);
    }
  }
  /**
   * operator to find new system (H1 + H2)x = b1+b2
   * @param other linear system to add (H2)
   * @return  sum of 2 systems
   */
  NormalLinearSystem operator+(const NormalLinearSystem &other) {
    NormalLinearSystem result = *this;
    result.H += other.H;
    result.b += other.b;
    return result;
  }
  /**
   * operator to find new system (H1 + H2)x = b1+b2
   * @param other linear system to add (H2)
   */
  void operator+=(const NormalLinearSystem &other) {
    H += other.H;
    b += other.b;
  }
  /**
   * @brief adds system to block, can be viewed as adding a sparse system
   *
   *  | H11 H12 H13 | b1 |    | 0  0  0 | 0 |
   *  |     H22 H23 | b2 | +  |    H  0 | b |
   *  |         H33 | b3 |    |       0 | 0 |
   *
   * @param offset number of rows and colls to skip from top left corner
   * @param other system to add (H, b)
   */
  template <typename ScalarOther, int SizeOther>
  void addToBlock(int offset, const NormalLinearSystem<ScalarOther, SizeOther> &other) {
    static_assert(SizeOther != Eigen::Dynamic);
    static_assert(std::is_same_v<Scalar, ScalarOther>);
    H.template block<SizeOther, SizeOther>(offset, offset) += other.H;
    b.template segment<SizeOther>(offset) += other.b;
  }
  /**
   * multiplies system by a scalar (H * s) x = b * S
   * @param scalar scalar to multiply system with (s)
   * @return scaled system
   */
  NormalLinearSystem operator*(Scalar scalar) {
    NormalLinearSystem result = *this;
    result.H *= scalar;
    result.b *= scalar;
    return result;
  }
  /**
   * operator to find new system (H1 - H2)x = b1 - b2
   * @param other linear system to add (H2)
   * @return  difference of 2 systems
   */
  NormalLinearSystem operator-(const NormalLinearSystem &other) {
    NormalLinearSystem result = *this;
    result.H -= other.H;
    result.b -= other.b;
    return result;
  }
  /**
   * reduces system via schur complement
   * transforms system
   * [A B^T]  x  =  b
   * [B C  ]  y  =  v
   *
   * to
   *
   * (A - B^T * C^-1 * B) * x = b - B^T * C^-1 * v
   * @param indices_to_eliminate vector that defines rows and columns to be eliminated (matrix C)
   */
  void reduce_system(std::vector<int> indices_to_eliminate);

  /**
   * solves system
   * @return solution of Hx = b
   */
  Eigen::Vector<Scalar, SIZE> solve();

  /** hold H matrix of the system H*x=b*/
  Eigen::Matrix<Scalar, SIZE, SIZE> H;
  /** hold b vector of the system H*x=b*/
  Eigen::Vector<Scalar, SIZE> b;
};
}  // namespace energy
}  // namespace dsopp

#endif
