#ifndef DSOPP_EPIPOLAR_LINE_BUILDER_HPP
#define DSOPP_EPIPOLAR_LINE_BUILDER_HPP

#include <queue>

#include "common/settings.hpp"
#include "energy/epipolar_geometry/epipolar_line.hpp"
#include "energy/epipolar_geometry/general_epipolar_line_triangulator.hpp"
#include "energy/motion/motion.hpp"
#include "energy/projector/camera_projector.hpp"
#include "energy/projector/camera_reproject.hpp"

namespace dsopp {
namespace energy {
namespace epipolar_geometry {

/**
 * \brief base class for building epipolar line
 *
 * @tparam Derived derived class
 */
template <class Derived>
class EpipolarLineBuilderBase {
 public:
  /**
   *
   * @param observed_point point that defines epipolar line for target frame camera
   * @return epipolar line
   */
  EpipolarLine build(const Eigen::Vector2<Precision> &observed_point) const {
    return static_cast<Derived const *>(this)->buildSegment(observed_point, 0, kMaxIdepth);
  }

 protected:
  /** maximum inverse depth */
  const Precision kMaxIdepth = 1000;
};
/**
 * \brief General class for creating epipolar line for generic motion and camera models.
 *
 * Creates image of defined by `observed point` epipolar line on the target frame
 *
 * @tparam Model camera model type
 * @tparam Motion motion type
 */
template <class Model, motion::MotionProduct MotionProduct>
class EpipolarLineBuilder : public EpipolarLineBuilderBase<EpipolarLineBuilder<Model, MotionProduct>> {
 public:
  /**
   *
   * @param model camera model
   * @param t_t_r transformation from reference frame to target
   */
  EpipolarLineBuilder(const Model &model, const MotionProduct &t_t_r) : model_(model), t_t_r_(t_t_r) {}
  /**
   *
   * projectrs line (in target coordinate system)
   * `alpha` `point_min` + (1 - 'alpha`) `point_max` onto target image
   *
   * @param observed_point point that defines epipolar line for target frame camera
   * @param idepthmin inverse depth minimum
   * @param idepthmax inverse depth maximum
   * @return epipolar line image onto target frame
   */
  EpipolarLine buildSegment(const Eigen::Vector2<Precision> &observed_point, Precision idepthmin,
                            Precision idepthmax) const {
    reprojection::Projector<Precision, Model, MotionProduct> projector(model_, t_t_r_);
    Eigen::Vector4<Precision> point_min, point_max;

    // I. Projecting points to the target camera coordinate system
    projector.unproject(observed_point, idepthmin, point_min);
    projector.unproject(observed_point, idepthmax, point_max);

    point_min = projector.transform(point_min);
    point_max = projector.transform(point_max);

    // II. Validating inverse depthes
    Precision point_min_idepth = point_min(3) / point_min(2);
    Precision point_max_idepth = point_max(3) / point_max(2);

    if (point_min_idepth < 0 && point_max_idepth < 0) return {};

    Eigen::Vector4<Precision> direction = point_max - point_min;

    // III. Searching the point to start building image of the epipolar line
    const Precision distance = direction.norm() / 2;
    direction.normalize();

    using SearchingPair = std::pair<Eigen::Vector4<Precision>, Precision>;
    std::queue<SearchingPair> search;
    search.push({point_min + distance * direction, distance / 2});

    bool success = false;
    Eigen::Vector4<Precision> starting_point = point_max;
    Eigen::Vector2<Precision> pixel;

    if (projector.project(point_min, pixel)) {
      starting_point = point_min;
      success = true;
    }
    if (projector.project(point_max, pixel)) {
      starting_point = point_max;
      success = true;
    }

    const Precision kDistEps = 1e-3_p * distance;

    while (!search.empty() && !success) {
      SearchingPair point = search.front();
      search.pop();
      if (projector.project(point.first, pixel)) {
        Eigen::Vector2<Precision> init;
        success = true;
        starting_point = point.first;
      } else if (point.second > 2 * kDistEps) {
        search.push({point.first - point.second * direction, point.second / 2});
        search.push({point.first + point.second * direction, point.second / 2});
      }
    }

    if (!success) {
      return {};
    }

    EpipolarLine line;

    // IV. Adding points to the left from starting_point
    {
      auto points = fillLine(starting_point, point_min, projector, observed_point);
      for (auto it = points.rbegin(); it != points.rend(); ++it) {
        line.points.push_back(*it);
      }
    }
    // V. Adding point to the right from starting_point
    {
      auto points = fillLine(starting_point, point_max, projector, observed_point);
      if (!points.empty()) line.points.insert(line.points.end(), points.begin() + 1, points.end());
    }

    return line;
  }

  /**
   * creates triangulator object
   *
   * @param point_reference point on reference frame
   * @return triangulator object
   */
  EpipolarLineTriangulator<Model, MotionProduct> getTriangulator(
      const Eigen::Vector2<Precision> &point_reference) const {
    return EpipolarLineTriangulator<Model, MotionProduct>(model_, t_t_r_, point_reference);
  }

 private:
  /**
   * Fills epipolar line (`line`) with image of points between `start` and `stop` with 1px distance.
   *
   * @param start starting point of the line
   * @param stop stopping point of the line
   * @param projector generalized point projector
   * @param reference_point observed point on reference frame
   * @return vector of epipolar points
   */
  std::vector<EpipolarLine::EpipolarLinePoint> fillLine(
      const Eigen::Vector4<Precision> &start, const Eigen::Vector4<Precision> &stop,
      const reprojection::Projector<Precision, Model, MotionProduct> &projector,
      const Eigen::Vector2<Precision> &reference_point) const {
    Eigen::Vector4<Precision> current_point = start;

    Eigen::Vector2<Precision> pixel;

    Eigen::Matrix<Precision, 2, 4> projection_jacobian;

    std::vector<EpipolarLine::EpipolarLinePoint> points;
    while (projector.project(current_point, pixel, projection_jacobian)) {
      /** if we are not moving anywhere */
      if (!points.empty() && (pixel - points.back().projection).norm() < 0.1) return points;

      Precision idepth = projector.evalReferenceIdepth(current_point, reference_point);
      Precision target_idepth = projector.evalTargetIdepth(current_point, pixel);

      if (model_.validIdepth(idepth) && model_.validIdepth(target_idepth)) {
        points.push_back(EpipolarLine::EpipolarLinePoint(pixel, idepth, target_idepth));
      }

      Eigen::Vector2<Precision> alpha_jacobian = projection_jacobian * (stop - current_point);
      Precision alpha_jacobian_norm = alpha_jacobian.norm();

      if (alpha_jacobian_norm < 1e-12) return points;

      Precision delta_alpha = 1 / alpha_jacobian_norm;
      if (delta_alpha >= 1) {
        return points;
      }
      current_point = (1 - delta_alpha) * current_point + delta_alpha * stop;
    }

    return points;
  }

  /** camera model */
  const Model &model_;
  /** transformation from reference to target */
  const MotionProduct &t_t_r_;
};  // namespace epipolar_geometry

}  // namespace epipolar_geometry
}  // namespace energy
}  // namespace dsopp

#include "energy/epipolar_geometry/epipolar_line_builder_pinhole_se3.hpp"

#endif  // DSOPP_EPIPOLAR_LINE_BUILDER_HPP
