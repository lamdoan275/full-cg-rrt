/**
 * *********************************************************
 *
 * @file: evaluator.h
 * @brief: Orchestrates registered path/tracking metrics (DIP root: depends
 *         only on IPathMetric/ITrackingMetric, never a concrete metric)
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef RMP_PATH_EVALUATOR_EVALUATOR_H_
#define RMP_PATH_EVALUATOR_EVALUATOR_H_

#include <memory>
#include <vector>

#include "path_evaluator/metric.h"

namespace rmp::path_evaluator {

/**
 * @brief Runs every registered metric over a path (and, separately, over a
 *        reference path + actual trajectory pair). Adding a new metric only
 *        requires registering an instance -- this class never changes (OCP).
 */
class PathEvaluator {
public:
  void registerMetric(std::shared_ptr<IPathMetric> metric);
  void registerMetric(std::shared_ptr<ITrackingMetric> metric);

  std::vector<MetricResult> evaluatePath(const common::geometry::Points3d& path) const;
  std::vector<MetricResult> evaluateTracking(
      const common::geometry::Points3d& reference_path,
      const common::geometry::Points3d& actual_trajectory) const;

private:
  std::vector<std::shared_ptr<IPathMetric>> path_metrics_;
  std::vector<std::shared_ptr<ITrackingMetric>> tracking_metrics_;
};

}  // namespace rmp::path_evaluator
#endif  // RMP_PATH_EVALUATOR_EVALUATOR_H_
