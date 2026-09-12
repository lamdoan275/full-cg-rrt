/**
 * *********************************************************
 *
 * @file: tracking_error_metric.h
 * @brief: Cross-track error between a reference path and the actual
 *         executed trajectory
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef RMP_PATH_EVALUATOR_METRICS_TRACKING_ERROR_METRIC_H_
#define RMP_PATH_EVALUATOR_METRICS_TRACKING_ERROR_METRIC_H_

#include "path_evaluator/metric.h"

namespace rmp::path_evaluator {

/**
 * @brief For each point of the actual trajectory, the perpendicular
 *        distance to the nearest segment of the reference path (clamped to
 *        the segment). Reports RMSE as the headline value, plus mean/max.
 *        Spatial (not time-parametrized): matches planners in this repo,
 *        which emit a static path with no velocity profile.
 */
class TrackingErrorMetric : public ITrackingMetric {
public:
  std::string name() const override;
  MetricResult evaluate(const common::geometry::Points3d& reference_path,
                        const common::geometry::Points3d& actual_trajectory) const override;
};

}  // namespace rmp::path_evaluator
#endif  // RMP_PATH_EVALUATOR_METRICS_TRACKING_ERROR_METRIC_H_
