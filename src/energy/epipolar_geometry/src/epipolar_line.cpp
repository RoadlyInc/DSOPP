#include "energy/epipolar_geometry/epipolar_line.hpp"

namespace dsopp {
namespace energy {
namespace epipolar_geometry {

EpipolarLine::EpipolarLinePoint::EpipolarLinePoint(const Eigen::Vector2<Precision> &_projection,
                                                   Precision _reference_idepth, Precision _target_idepth)
    : projection(_projection), reference_idepth(_reference_idepth), target_idepth(_target_idepth) {}

bool EpipolarLine::empty() { return points.empty(); }

void EpipolarLine::addPoint(const Eigen::Vector2<Precision> &projection, Precision idepth, Precision target_idepth) {
  points.emplace_back(projection, idepth, target_idepth);
}

EpipolarLine::EpipolarLinePoint EpipolarLine::shift(size_t idx, Precision step) const {
  if (idx == 0) {
    step /= (points[idx].projection - points[idx + 1].projection).norm();
  } else if (idx == points.size() - 1) {
    step /= (points[idx - 1].projection - points[idx].projection).norm();
  } else {
    step /= ((points[idx - 1].projection - points[idx + 1].projection) / 2).norm();
  }

  int idx_step = static_cast<int>(round(step));

  Precision subpixel_shift = step - static_cast<Precision>(idx_step);

  int idx_signed = static_cast<int>(idx) + idx_step;

  // interpolate liner
  if (idx_signed <= 0) {
    Precision alpha = step + static_cast<Precision>(idx);
    auto p0 = points[0];
    auto p1 = points[1];
    return {p0.projection + alpha * (p1.projection - p0.projection),
            p0.reference_idepth + alpha * (p1.reference_idepth - p0.reference_idepth),
            p0.target_idepth + alpha * (p1.target_idepth - p0.target_idepth)};
  }
  if (idx_signed >= static_cast<int>(points.size()) - 1) {
    Precision alpha = (step + static_cast<Precision>(idx) - (static_cast<Precision>(points.size()) - 1));
    auto p0 = points.back();
    auto p1 = points[(points.size() - 2)];

    return {p0.projection + alpha * (p0.projection - p1.projection),
            p0.reference_idepth + alpha * (p0.reference_idepth - p1.reference_idepth),
            p0.target_idepth + alpha * (p0.target_idepth - p1.target_idepth)};
  }

  int neighbour_idx_shift = subpixel_shift > 0 ? 1 : -1;
  subpixel_shift = abs(subpixel_shift);

  auto pt1 = points[static_cast<size_t>(idx_signed)];
  auto pt2 = points[static_cast<size_t>(idx_signed + neighbour_idx_shift)];
  Precision a1 = (1 - subpixel_shift);
  Precision a2 = subpixel_shift;
  // TODO rewrite it to find idepth on reference frame more precisely
  return {a1 * pt1.projection + a2 * pt2.projection, a1 * pt1.reference_idepth + a2 * pt2.reference_idepth,
          a1 * pt1.target_idepth + a2 * pt2.target_idepth};
}

Eigen::Vector2<Precision> EpipolarLine::tangent(size_t idx) const {
  int left_idx = std::max(0, static_cast<int>(idx) - 1);
  int right_idx = std::min(static_cast<int>(points.size()) - 1, static_cast<int>(idx) + 1);

  return points[static_cast<size_t>(right_idx)].projection - points[static_cast<size_t>(left_idx)].projection;
}

Precision EpipolarLine::length() {
  if (length_ < 0) {
    length_ = 0;
    for (size_t i = 1; i < points.size(); i++) {
      length_ += (points[i].projection - points[i - 1].projection).norm();
    }
  }
  return length_;
}

}  // namespace epipolar_geometry
}  // namespace energy
}  // namespace dsopp
