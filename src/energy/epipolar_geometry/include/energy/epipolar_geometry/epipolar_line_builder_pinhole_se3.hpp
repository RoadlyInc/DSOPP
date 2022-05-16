#ifndef DSOPP_EPIPOLAR_LINE_PINHOLE_SE3_HPP
#define DSOPP_EPIPOLAR_LINE_PINHOLE_SE3_HPP

#include <Eigen/Dense>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/epipolar_geometry/epipolar_line.hpp"
#include "energy/epipolar_geometry/se3_epipolar_line_triangulator.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/projector/camera_reproject.hpp"

namespace dsopp {
namespace energy {
namespace epipolar_geometry {
namespace {
/**
 * creates pinhole camera epipolarLine from the number of points and the equation coefficients.
 * The epipolar line equation is represented as
 * a / (x + b) + c + d * x = idepth. Depending on the values of the coefficients this equation could be
 * linear or hyperbolic
 * @param size number of point in the line
 * @param equation_coefficients coefficients of the depth equation
 */
void makePinholeCameraEpipolarLine(const size_t size, Eigen::Vector2<Precision> start, Eigen::Vector2<Precision> end,
                                   EpipolarLine &epipolar_line, const EpipolarLineTriangulatorSE3 &triangulator) {
  auto step = (end - start) / size;
  Eigen::Vector2<Precision> point_new = start - step;
  for (size_t current_position = 0; current_position <= size; current_position++) {
    point_new += step;
    epipolar_line.addPoint(point_new, triangulator.getInverseDepth(point_new), 0);
  }
}
/**
 * function for finding the intersection of the vertical line with the borders of the image
 *
 * @param x x coordinate of the vertical line
 * @param image_size width and height of the image
 * @param[out] point_start,point_end points of the image
 * @return true if the epipolar line intersects with the image plane and false otherwise
 */
bool createVerticalLine(Precision x, const Eigen::Vector2<Precision> &image_size,
                        Eigen::Vector2<Precision> &point_start, Eigen::Vector2<Precision> &point_end,
                        Precision border_size) {
  point_start << x, border_size;
  point_end << x, image_size(1) - 1 - border_size;
  return x >= border_size and x <= (image_size(0) - 1 - border_size);
}
/**
 * function for finding the intersection of the general line with the borders of the image
 *
 * @param k,s line coefficients in the equation y = k * x + s
 * @param image_size width and height of the image
 * @param[out] point_start,point_end points of the image
 * @return true if the epipolar line intersects with the image plane and false otherwise
 */
bool createGeneralLine(Precision k, Precision s, const Eigen::Vector2<Precision> &image_size,
                       Eigen::Vector2<Precision> &point_start, Eigen::Vector2<Precision> &point_end,
                       Precision border_size) {
  // horizontal line
  if (k == 0) {
    point_start << border_size, s;
    point_end << image_size(0) - 1 - border_size, s;
    return s >= border_size and s <= (image_size(1) - 1 - border_size);
  }
  Precision x_min = border_size;
  Precision y_min = border_size;
  Precision x_max = image_size(0) - 1 - border_size;
  Precision y_max = image_size(1) - 1 - border_size;
  // function y(x) = k * x + s limited by image boundaries
  auto y_x = [=](Precision x) { return k * x + s; };
  // function x(y) = y / k - s / k limited by image boundaries
  auto x_y = [=](Precision y) { return y / k - s / k; };
  Precision y_start = std::clamp(y_x(x_min), y_min, y_max);
  Precision y_end = std::clamp(y_x(x_max), y_min, y_max);
  Precision x_start = std::clamp(x_y(y_min), x_min, x_max);
  Precision x_end = std::clamp(x_y(y_max), x_min, x_max);
  if (k < 0) {
    std::swap(y_start, y_end);
  }
  point_start << x_start, y_start;
  point_end << x_end, y_end;
  if (y_x(x_min) < border_size and y_x(x_max) < border_size) {
    return false;
  }
  if (y_x(x_min) > (image_size(1) - 1 - border_size) and y_x(x_max) > (image_size(1) - 1 - border_size)) {
    return false;
  }
  if (x_y(y_min) < border_size and x_y(y_max) < border_size) {
    return false;
  }
  if (x_y(y_min) > (image_size(0) - 1 - border_size) and x_y(y_max) > (image_size(0) - 1 - border_size)) {
    return false;
  }
  return true;
}
/**
 * function for finding the intersection of the line, passing through two points, with the borders of the image
 *
 * @param point_1, point_2 points of the line
 * @param image_size width and height of the image
 * @param[out] point_left_border, point_right_border points of the intersection of the line with the image borders
 * @return true if the epipolar line intersects with the image plane and false otherwise
 */
bool intersectImageBorders(Eigen::Vector2<Precision> &point_1, Eigen::Vector2<Precision> &point_2,
                           const Eigen::Vector2<Precision> &image_size, Eigen::Vector2<Precision> &point_left_border,
                           Eigen::Vector2<Precision> &point_right_border, Precision border_size) {
  // The equation of a straight line passing through the two points (x_1, y_1) and (x_2, y_2)
  // is (y_1 - y_2) * x + (x_2 - x_1) * y + (x_1 * y_2 - x_2 * y_1) = 0
  // canonical equation of a line a * x + b * y + c = 0
  Eigen::Vector2<Precision> p_start, point_end;
  auto a = point_1(1) - point_2(1);
  auto b = point_2(0) - point_1(0);
  auto c = point_1(0) * point_2(1) - point_2(0) * point_1(1);
  if (a == 0 and b == 0) {  // single point
    return (point_1[0] >= border_size and point_1[1] >= border_size and
            point_1[0] <= (image_size[0] - 1 - border_size) and point_1[1] <= (image_size[1] - 1 - border_size));
  } else if (b == 0) {  // vertical line
    return createVerticalLine(point_1(0), image_size, point_left_border, point_right_border, border_size);
  } else {  // equation can be written y = k * x + s
    auto k = -(a / b);
    auto s = -(c / b);
    return createGeneralLine(k, s, image_size, point_left_border, point_right_border, border_size);
  }
}
/**
 * function for finding the number of points of the line
 *
 * @param point_start_depth, point_end_depth coordinates of the points with max and min values of the inverse depth
 * @return number of points of the line
 */
size_t getSize(const Eigen::Vector2<Precision> &point_start_depth, const Eigen::Vector2<Precision> &point_end_depth) {
  const Precision kIncreaseNumber = 1;
  const size_t kMinPoints = 1;
  size_t size = static_cast<size_t>((point_end_depth - point_start_depth).norm() * kIncreaseNumber);
  return std::max(kMinPoints, size);
}
/**
 * function for сhecking that the point before and after the reprojection has a valid depth
 *
 * @param p_2d coordinates of the given pixel
 * @param idepth_1 inverse depth of the 3d point that projected on the given pixel in the first camera pose
 * @param t_target_reference homogeneous t_target_reference between camera positions
 * @param focal_lengths, principal_point intrinsic parameters of the camera
 * @return true if the point before and after the reprojection has a valid depth and false otherwise
 */

bool valid(const Eigen::Vector2<Precision> &p_2d, Precision idepth_1, const motion::SE3<Precision> &t_target_reference,
           const Eigen::Vector2<Precision> &focal_lengths, const Eigen::Vector2<Precision> &principal_point) {
  const Precision kMinDepth = model::PinholeCamera<Precision>::kMinDepth;
  const Precision kDepthEps = 1e-4_p;
  if (idepth_1 < 0 or idepth_1 > (1 / kMinDepth + kDepthEps)) {
    return false;
  }
  Eigen::Vector3<Precision> point_3d_1;
  // unproject
  point_3d_1.template head<2>() = (p_2d - principal_point).cwiseProduct(focal_lengths.cwiseInverse());
  point_3d_1[2] = 1;
  // transform
  Eigen::Vector4<Precision> point_3d_1_homogeneous;
  point_3d_1_homogeneous << point_3d_1, idepth_1;
  Eigen::Vector3<Precision> point_3d_2 = (t_target_reference * point_3d_1_homogeneous).template head<3>();

  Precision idepth_2 = 1 / point_3d_2[2];
  return idepth_2 >= 0 and idepth_1 <= (1 / kMinDepth + kDepthEps);
}
/**
 * function for replacing initial limits of the idepth (from 0 to 1 / kMinDepth) with the intersection with the
 * image plane
 *
 * @param idepth_limits_diff_validity true if the initial limits both valid (both have valid idepth before and after the
 * reprojection) and false otherwise
 * @param zero_depth_reprojected, inf_depth_reprojected true if points with initial limits of idepth (1 / kMinDepth and
 * 0) reprojected successfully and false otherwise
 * @param point_left_border, point_right_border coordinates of the intersections of the epipolar line with the imaginary
 * plane
 * @param idepth_borders inverse depth of the intersections of the epipolar line with the image plane
 * @param[out] point_start_depth, point_end_depth final borders of the epipolar line
 * @param[out] idepth_limits final borders of the inverse depth
 */
void findLineBorders(bool idepth_limits_diff_validity, bool zero_depth_reprojected, bool inf_depth_reprojected,
                     const Eigen::Vector2<Precision> &point_left_border,
                     const Eigen::Vector2<Precision> &point_right_border,
                     const Eigen::Vector2<Precision> &idepth_borders, Eigen::Vector2<Precision> &point_start_depth,
                     Eigen::Vector2<Precision> &point_end_depth, Eigen::Vector2<Precision> &idepth_limits) {
  // line splits into two parts
  if (idepth_limits_diff_validity) {
    if (zero_depth_reprojected and not inf_depth_reprojected) {
      if ((point_end_depth - point_start_depth).dot(point_right_border - point_left_border) > 0 and
          idepth_borders(0) > 0) {
        point_end_depth = point_left_border;
        idepth_limits(1) = idepth_borders(0);
      } else {
        point_end_depth = point_right_border;
        idepth_limits(1) = idepth_borders(1);
      }
    }
    if (inf_depth_reprojected and not zero_depth_reprojected) {
      if ((point_end_depth - point_start_depth).dot(point_right_border - point_left_border) > 0 and
          idepth_borders(1) > 0) {
        point_start_depth = point_right_border;
        idepth_limits(0) = idepth_borders(1);
      } else {
        point_start_depth = point_left_border;
        idepth_limits(0) = idepth_borders(0);
      }
    }
  } else {
    if (zero_depth_reprojected and not inf_depth_reprojected) {
      if ((point_end_depth - point_start_depth).dot(point_right_border - point_left_border) > 0 and
          idepth_borders(1) > 0) {
        point_end_depth = point_right_border;
        idepth_limits(1) = idepth_borders(1);
      } else {
        point_end_depth = point_left_border;
        idepth_limits(1) = idepth_borders(0);
      }
    }
    if (inf_depth_reprojected and not zero_depth_reprojected) {
      if ((point_end_depth - point_start_depth).dot(point_right_border - point_left_border) > 0 and
          idepth_borders(0) > 0) {
        point_start_depth = point_left_border;
        idepth_limits(0) = idepth_borders(0);
      } else {
        point_start_depth = point_right_border;
        idepth_limits(0) = idepth_borders(1);
      }
    }
  }
  if (not zero_depth_reprojected and not inf_depth_reprojected) {
    point_start_depth = point_left_border;
    point_end_depth = point_right_border;
    idepth_limits(0) = idepth_borders(0);
    idepth_limits(1) = idepth_borders(1);
  }
  // make it always rising
  if (idepth_limits(1) > idepth_limits(0)) {
    std::swap(idepth_limits(0), idepth_limits(1));
    std::swap(point_start_depth, point_end_depth);
  }
}
/**
 * function for checking all cases when the epipolar line cannot exist
 *
 * @param idepth_limits_diff_validity true if the initial limits both valid (both have valid idepth before and after the
 * reprojection) and false otherwise
 * @param zero_depth_reprojected, inf_depth_reprojected true if points with initial limits of idepth (1 / kMinDepth and
 * 0) reprojected successfully and false otherwise
 * @param idepth_borders_diff_validity true if the inverse depth of the intersections of the epipolar line with the
 * image plane are both valid and false otherwise
 * @param left_depth_reprojected_validity, right_depth_reprojected_validity validity of the left and right intersection
 * of the epipolar line with the image plane
 * @param idepth_limits borders of the inverse depth
 * @param idepthmin, idepthmax desired borders of the inverse depth
 */
bool epipolarLineNotExists(bool intersect_imaginary_plane, bool idepth_limits_diff_validity,
                           bool zero_depth_reprojected, bool inf_depth_reprojected, bool idepth_borders_diff_validity,
                           bool left_depth_reprojected_validity, bool right_depth_reprojected_validity,
                           const Eigen::Vector2<Precision> &idepth_limits, Precision idepthmin, Precision idepthmax) {
  // Epipolar line don't intersect with the image plane
  if (not intersect_imaginary_plane) {
    return true;
  }
  // Epipolar line splits into two parts and both lie out of the image plane
  if ((idepth_limits_diff_validity and (not zero_depth_reprojected) and (not inf_depth_reprojected)) and
      idepth_borders_diff_validity) {
    return true;
  }
  // Epipolar line is a solid line, but lies completely behind the image plane
  if ((not zero_depth_reprojected) and (not inf_depth_reprojected) and (not left_depth_reprojected_validity) and
      (not right_depth_reprojected_validity)) {
    return true;
  }
  // Epipolar line lies completely outside the given borders
  if (idepth_limits(1) > idepthmax or idepth_limits(0) < idepthmin) {
    return true;
  }
  // Given borders are not valid
  if (idepthmax < idepthmin) {
    return true;
  }
  return false;
}
}  // namespace
/**
 * \brief epipolar line builder specialization for `PinholeCamera` and `SE3` motion
 */
template <>
class EpipolarLineBuilder<model::PinholeCamera<Precision>, motion::SE3<Precision>>
    : public EpipolarLineBuilderBase<EpipolarLineBuilder<model::PinholeCamera<Precision>, motion::SE3<Precision>>> {
 public:
  /**
   *
   * @param model camera model
   * @param t_t_r transformation from refernce to target frame
   */
  EpipolarLineBuilder(const model::PinholeCamera<Precision> &model, const motion::SE3<Precision> &t_t_r)
      : model_(model), t_t_r_(t_t_r) {}

  /**
   *
   * @param observed_point point on reference frame
   * @param idepthmin inverse depth minimum
   * @param idepthmax inverse depth maximum
   * @return epipolar line image onto target frame
   */
  EpipolarLine buildSegment(const Eigen::Vector2<Precision> &observed_point, Precision idepthmin,
                            Precision idepthmax) const {
    EpipolarLine epipolar_line;
    if (t_t_r_.translation().norm() < 1. / this->kMaxIdepth) {
      return epipolar_line;
    }

    Eigen::Vector2<Precision> point_start_depth, point_end_depth, point_left_border, point_right_border;
    Eigen::Vector2<Precision> idepth_limits = {this->kMaxIdepth, 0};

    reprojection::ArrayReprojector<Precision, model::PinholeCamera<Precision>, motion::SE3<Precision>> reprojector(
        model_, t_t_r_);

    bool zero_depth_reprojected = reprojector.reproject(observed_point, idepth_limits(0), point_start_depth);
    bool inf_depth_reprojected = reprojector.reproject(observed_point, idepth_limits(1), point_end_depth);

    bool zero_depth_reprojected_validity =
        valid(observed_point, idepth_limits(0), t_t_r_, model_.focal_lengths(), model_.principal_point());
    bool inf_depth_reprojected_validity =
        valid(observed_point, idepth_limits(1), t_t_r_, model_.focal_lengths(), model_.principal_point());
    bool idepth_limits_diff_validity = zero_depth_reprojected_validity ^ inf_depth_reprojected_validity;

    point_left_border.setZero();
    point_right_border.setZero();
    auto triangulator = getTriangulator(observed_point);
    bool intersect_imaginary_plane = intersectImageBorders(point_start_depth, point_end_depth, model_.image_size(),
                                                           point_left_border, point_right_border, model_.kBorderSize);
    Eigen::Vector2<Precision> idepth_borders = {triangulator.getInverseDepth(point_left_border),
                                                triangulator.getInverseDepth(point_right_border)};

    bool left_depth_reprojected_validity =
        valid(observed_point, idepth_borders(0), t_t_r_, model_.focal_lengths(), model_.principal_point());
    bool right_depth_reprojected_validity =
        valid(observed_point, idepth_borders(1), t_t_r_, model_.focal_lengths(), model_.principal_point());
    bool idepth_borders_diff_validity = left_depth_reprojected_validity ^ right_depth_reprojected_validity;

    // one point epipolar line
    if (point_start_depth.isApprox(point_end_depth)) {
      if (left_depth_reprojected_validity) {
        epipolar_line.points.push_back(EpipolarLine::EpipolarLinePoint{point_start_depth, 0, 0});
      }
      return epipolar_line;
    }

    findLineBorders(idepth_limits_diff_validity, zero_depth_reprojected, inf_depth_reprojected, point_left_border,
                    point_right_border, idepth_borders, point_start_depth, point_end_depth, idepth_limits);

    if (epipolarLineNotExists(intersect_imaginary_plane, idepth_limits_diff_validity, zero_depth_reprojected,
                              inf_depth_reprojected, idepth_borders_diff_validity, left_depth_reprojected_validity,
                              right_depth_reprojected_validity, idepth_limits, idepthmin, idepthmax)) {
      return epipolar_line;
    }

    if (idepth_limits(0) > idepthmax) {
      idepth_limits(0) = idepthmax;
      reprojector.reproject(observed_point, idepth_limits(0), point_start_depth);
    }
    if (idepth_limits(1) < idepthmin) {
      idepth_limits(1) = idepthmin;
      reprojector.reproject(observed_point, idepth_limits(1), point_end_depth);
    }

    size_t size = getSize(point_start_depth, point_end_depth);
    makePinholeCameraEpipolarLine(size, point_end_depth, point_start_depth, epipolar_line, triangulator);
    return epipolar_line;
  }
  /**
   * creates triangulator object
   *
   * @param point_reference point on reference frame
   * @return triangulator object
   */
  EpipolarLineTriangulatorSE3 getTriangulator(const Eigen::Vector2<Precision> &point_reference) const {
    return EpipolarLineTriangulatorSE3(t_t_r_, model_.focal_lengths(), model_.principal_point(), point_reference,
                                       this->kMaxIdepth);
  }

 private:
  /** camera model */
  const model::PinholeCamera<Precision> &model_;
  /** transformation from refernce to target frame */
  const motion::SE3<Precision> &t_t_r_;
};
}  // namespace epipolar_geometry
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_EPIPOLAR_LINE_PINHOLE_SE3_HPP
