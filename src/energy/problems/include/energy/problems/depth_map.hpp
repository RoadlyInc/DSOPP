#ifndef DSOPP_DEPTH_MAP_HPP
#define DSOPP_DEPTH_MAP_HPP

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp::energy::problem {

/**
 * Depth map corresponding to the image
 */
struct DepthMap {
  /**
   * Variation-weighted depth
   */
  struct WeightedIdepth {
    /** inverse depth */
    Precision idepth = 0;
    /** weight */
    Precision weight = 0;
    /**
     * sum operator overloading
     * @param rhs idepth to sum
     * @return sum
     * */
    WeightedIdepth operator+(const WeightedIdepth &rhs) const {
      return WeightedIdepth{idepth + rhs.idepth, weight + rhs.weight};
    }
  };
  /**
   * Creating depth map with given size
   * @param width, height given sizes
   */
  DepthMap(long width, long height) : map(width, height) {}
  /** depth map */
  Eigen::MatrixX<WeightedIdepth> map;
};

}  // namespace dsopp::energy::problem

#endif  // DSOPP_DEPTH_MAP_HPP
