#ifndef DSOPP_EPIPOLAR_LINE_HPP
#define DSOPP_EPIPOLAR_LINE_HPP

#include <Eigen/Dense>
#include <vector>
#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace energy {
namespace epipolar_geometry {
/**
 * EpipolarLine represent sorted set of points on epipolar line
 */
struct EpipolarLine {
  /** Point of an epipolar line */
  struct EpipolarLinePoint {
    /**
     * Creates point on the epipolar line
     *
     * @param _projection projection of the point
     * @param _reference_idepth inverse depth of the point
     * @param _target_idepth target inverse depth
     */
    EpipolarLinePoint(const Eigen::Vector2<Precision> &_projection, Precision _reference_idepth,
                      Precision _target_idepth);
    /** projection of the point */
    Eigen::Vector2<Precision> projection;
    /** inverse depth of the point on reference frame */
    Precision reference_idepth;
    /**  inverse depth of the point on target frame */
    Precision target_idepth;
  };
  /**
   * @return true if there are no points in the epipolar line
   */
  bool empty();
  /**
   * Add new point to storage
   *
   * @param projection projection of the point
   * @param idepth inverse depth
   * @param target_idepth target idepth
   */
  void addPoint(const Eigen::Vector2<Precision> &projection, Precision idepth, Precision target_idepth);

  /**
   * find tangent to point at position `idx`
   * @param idx point index
   * @return tangent at point
   */
  Eigen::Vector2<Precision> tangent(size_t idx) const;

  /**
   * @return length of the epipolar line
   */
  Precision length();

  /**
   *  shifts `points[idx]` on the epipolar line by `step` pixels
   *
   * @param idx point index in epipolar line `points` container
   * @param step step to the new point
   * @return new point on epipolar line
   */
  EpipolarLinePoint shift(size_t idx, Precision step) const;
  /** storage of points */
  std::vector<EpipolarLinePoint> points;

 private:
  /** length of the epipolar line in pixels */
  Precision length_ = -1;
};

}  // namespace epipolar_geometry
}  // namespace energy
}  // namespace dsopp
#endif  // DSOPP_EPIPOLAR_LINE_HPP
