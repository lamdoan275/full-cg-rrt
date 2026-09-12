/**
 * *********************************************************
 *
 * @file: smoothness_metric.h
 * @brief: Arc-length-normalized heading change (discrete curvature) of a
 *         planned path
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef RMP_PATH_EVALUATOR_METRICS_SMOOTHNESS_METRIC_H_
#define RMP_PATH_EVALUATOR_METRICS_SMOOTHNESS_METRIC_H_

#include "path_evaluator/metric.h"

namespace rmp::path_evaluator {

/**
 * @brief Discrete curvature (kappa = dtheta/ds) based smoothness metric.
 *        Reports mean(|kappa|) as the headline value and max(|kappa|) as a
 *        detail (catches a single sharp turn that a mean would dilute).
 */
class SmoothnessMetric : public IPathMetric {
public:
  std::string name() const override;
  MetricResult evaluate(const common::geometry::Points3d& path) const override;
};

}  // namespace rmp::path_evaluator
#endif  // RMP_PATH_EVALUATOR_METRICS_SMOOTHNESS_METRIC_H_
