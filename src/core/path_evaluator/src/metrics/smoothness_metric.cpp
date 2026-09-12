/**
 * *********************************************************
 *
 * @file: smoothness_metric.cpp
 * @brief: Arc-length-normalized heading change (discrete curvature) of a
 *         planned path
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#include <cmath>

#include "path_evaluator/metrics/smoothness_metric.h"

using rmp::common::geometry::Points3d;

namespace rmp::path_evaluator {
namespace {
double wrapToPi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

std::string SmoothnessMetric::name() const {
  return "smoothness";
}

MetricResult SmoothnessMetric::evaluate(const Points3d& path) const {
  MetricResult result;
  result.name = name();
  result.details["max_curvature"] = 0.0;

  // need at least 3 points to have a heading change (2 segments)
  if (path.size() < 3) return result;

  double sum_abs_kappa = 0.0;
  double max_abs_kappa = 0.0;
  int n = 0;

  double prev_theta = std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    double theta = std::atan2(path[i + 1].y() - path[i].y(), path[i + 1].x() - path[i].x());
    double d_theta = wrapToPi(theta - prev_theta);
    double ds = std::hypot(path[i].x() - path[i - 1].x(), path[i].y() - path[i - 1].y());

    if (ds > 1e-9) {
      double kappa = std::abs(d_theta / ds);
      sum_abs_kappa += kappa;
      max_abs_kappa = std::max(max_abs_kappa, kappa);
      ++n;
    }
    prev_theta = theta;
  }

  result.value = (n > 0) ? sum_abs_kappa / n : 0.0;
  result.details["mean_curvature"] = result.value;
  result.details["max_curvature"] = max_abs_kappa;
  return result;
}

}  // namespace rmp::path_evaluator
