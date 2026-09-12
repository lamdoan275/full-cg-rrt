/**
 * *********************************************************
 *
 * @file: report.h
 * @brief: Value types returned by path/tracking metrics
 * @author: Nam
 * @date: 2026-09-10
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef RMP_PATH_EVALUATOR_REPORT_H_
#define RMP_PATH_EVALUATOR_REPORT_H_

#include <map>
#include <string>

namespace rmp::path_evaluator {

/**
 * @brief Result of a single metric evaluation
 */
struct MetricResult {
  std::string name;                       // e.g. "smoothness.mean_curvature"
  double value = 0.0;
  std::map<std::string, double> details;  // optional breakdown (e.g. max, mean)
};

}  // namespace rmp::path_evaluator
#endif  // RMP_PATH_EVALUATOR_REPORT_H_
