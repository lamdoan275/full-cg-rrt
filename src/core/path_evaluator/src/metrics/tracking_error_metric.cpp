/**
 * *********************************************************
 *
 * @file: tracking_error_metric.cpp
 * @brief: Cross-track error between a reference path and the actual
 *         executed trajectory
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#include <algorithm>
#include <cmath>
#include <limits>

#include "path_evaluator/metrics/tracking_error_metric.h"

using rmp::common::geometry::Point3d;
using rmp::common::geometry::Points3d;

namespace rmp::path_evaluator {
namespace {
// Distance from `p` to the closest point on segment [a, b], clamping the
// projection to the segment so points beyond an endpoint measure to that
// endpoint rather than to an infinite extension of the line.
double distanceToSegment(const Point3d& p, const Point3d& a, const Point3d& b) {
  double vx = b.x() - a.x(), vy = b.y() - a.y();
  double wx = p.x() - a.x(), wy = p.y() - a.y();

  double len_sq = vx * vx + vy * vy;
  double t = (len_sq > 1e-12) ? std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0) : 0.0;

  double proj_x = a.x() + t * vx, proj_y = a.y() + t * vy;
  return std::hypot(p.x() - proj_x, p.y() - proj_y);
}

double distanceToPath(const Point3d& p, const Points3d& path) {
  if (path.empty()) return 0.0;
  if (path.size() == 1) return std::hypot(p.x() - path[0].x(), p.y() - path[0].y());

  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    min_dist = std::min(min_dist, distanceToSegment(p, path[i], path[i + 1]));
  }
  return min_dist;
}
}  // namespace

std::string TrackingErrorMetric::name() const {
  return "tracking_error";
}

MetricResult TrackingErrorMetric::evaluate(const Points3d& reference_path,
                                           const Points3d& actual_trajectory) const {
  MetricResult result;
  result.name = name();
  result.details["max_error"] = 0.0;
  result.details["mean_error"] = 0.0;

  if (reference_path.empty() || actual_trajectory.empty()) return result;

  double sum_sq = 0.0, sum = 0.0, max_error = 0.0;
  for (const auto& point : actual_trajectory) {
    double e = distanceToPath(point, reference_path);
    sum_sq += e * e;
    sum += e;
    max_error = std::max(max_error, e);
  }

  const size_t n = actual_trajectory.size();
  result.value = std::sqrt(sum_sq / n);
  result.details["max_error"] = max_error;
  result.details["mean_error"] = sum / n;
  return result;
}

}  // namespace rmp::path_evaluator
