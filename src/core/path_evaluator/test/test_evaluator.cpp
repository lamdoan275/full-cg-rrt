#include <gtest/gtest.h>

#include "path_evaluator/evaluator.h"
#include "path_evaluator/metrics/smoothness_metric.h"
#include "path_evaluator/metrics/tracking_error_metric.h"

using rmp::common::geometry::Points3d;
using rmp::path_evaluator::PathEvaluator;
using rmp::path_evaluator::SmoothnessMetric;
using rmp::path_evaluator::TrackingErrorMetric;

TEST(PathEvaluator, RunsRegisteredPathMetrics) {
  PathEvaluator evaluator;
  evaluator.registerMetric(std::make_shared<SmoothnessMetric>());

  Points3d path{ { 0, 0 }, { 1, 0 }, { 2, 0 } };
  auto results = evaluator.evaluatePath(path);

  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].name, "smoothness");
}

TEST(PathEvaluator, RunsRegisteredTrackingMetrics) {
  PathEvaluator evaluator;
  evaluator.registerMetric(std::make_shared<TrackingErrorMetric>());

  Points3d path{ { 0, 0 }, { 10, 0 } };
  auto results = evaluator.evaluateTracking(path, path);

  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].name, "tracking_error");
  EXPECT_NEAR(results[0].value, 0.0, 1e-9);
}

TEST(PathEvaluator, PathMetricsDoNotLeakIntoTrackingResults) {
  // OCP/ISP check: registering a path-only metric must not appear when
  // evaluating tracking, and vice versa -- the two registries stay separate.
  PathEvaluator evaluator;
  evaluator.registerMetric(std::make_shared<SmoothnessMetric>());
  evaluator.registerMetric(std::make_shared<TrackingErrorMetric>());

  Points3d path{ { 0, 0 }, { 1, 0 }, { 2, 0 } };
  EXPECT_EQ(evaluator.evaluatePath(path).size(), 1u);
  EXPECT_EQ(evaluator.evaluateTracking(path, path).size(), 1u);
}

TEST(PathEvaluator, NoRegisteredMetricsReturnsEmpty) {
  PathEvaluator evaluator;
  EXPECT_TRUE(evaluator.evaluatePath({ { 0, 0 } }).empty());
  EXPECT_TRUE(evaluator.evaluateTracking({ { 0, 0 } }, { { 0, 0 } }).empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
