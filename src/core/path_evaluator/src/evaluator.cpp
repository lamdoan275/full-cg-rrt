/**
 * *********************************************************
 *
 * @file: evaluator.cpp
 * @brief: Orchestrates registered path/tracking metrics
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#include "path_evaluator/evaluator.h"

using rmp::common::geometry::Points3d;

namespace rmp::path_evaluator {

void PathEvaluator::registerMetric(std::shared_ptr<IPathMetric> metric) {
  path_metrics_.push_back(std::move(metric));
}

void PathEvaluator::registerMetric(std::shared_ptr<ITrackingMetric> metric) {
  tracking_metrics_.push_back(std::move(metric));
}

std::vector<MetricResult> PathEvaluator::evaluatePath(const Points3d& path) const {
  std::vector<MetricResult> results;
  results.reserve(path_metrics_.size());
  for (const auto& metric : path_metrics_) {
    results.push_back(metric->evaluate(path));
  }
  return results;
}

std::vector<MetricResult> PathEvaluator::evaluateTracking(
    const Points3d& reference_path, const Points3d& actual_trajectory) const {
  std::vector<MetricResult> results;
  results.reserve(tracking_metrics_.size());
  for (const auto& metric : tracking_metrics_) {
    results.push_back(metric->evaluate(reference_path, actual_trajectory));
  }
  return results;
}

}  // namespace rmp::path_evaluator
