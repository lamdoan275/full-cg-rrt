/**
 * *********************************************************
 *
 * @file: metric.h
 * @brief: Abstract interfaces for path-quality metrics (ISP: a metric needs
 *         either just the planned path, or the path plus the actual
 *         executed trajectory)
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef RMP_PATH_EVALUATOR_METRIC_H_
#define RMP_PATH_EVALUATOR_METRIC_H_

#include <string>

#include "common/geometry/point.h"
#include "path_evaluator/report.h"

namespace rmp::path_evaluator {

/**
 * @brief Metric computed from a planned path alone (e.g. smoothness)
 */
class IPathMetric {
public:
  virtual ~IPathMetric() = default;

  /**
   * @brief Metric name, used as MetricResult::name
   */
  virtual std::string name() const = 0;

  /**
   * @brief Evaluate the metric over a planned path
   * @param path the planned path
   * @return the metric result
   */
  virtual MetricResult evaluate(const common::geometry::Points3d& path) const = 0;
};

/**
 * @brief Metric comparing a reference path against an actual executed
 *        trajectory (e.g. tracking error)
 */
class ITrackingMetric {
public:
  virtual ~ITrackingMetric() = default;

  /**
   * @brief Metric name, used as MetricResult::name
   */
  virtual std::string name() const = 0;

  /**
   * @brief Evaluate the metric over a reference path and the actual
   *        trajectory that was executed to follow it
   * @param reference_path    the planned path
   * @param actual_trajectory the actual trajectory (e.g. from odometry)
   * @return the metric result
   */
  virtual MetricResult evaluate(const common::geometry::Points3d& reference_path,
                                const common::geometry::Points3d& actual_trajectory) const = 0;
};

}  // namespace rmp::path_evaluator
#endif  // RMP_PATH_EVALUATOR_METRIC_H_
