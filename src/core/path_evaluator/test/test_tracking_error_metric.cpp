#include <cmath>

#include <gtest/gtest.h>

#include "path_evaluator/metrics/tracking_error_metric.h"

using rmp::common::geometry::Points3d;
using rmp::path_evaluator::TrackingErrorMetric;

TEST(TrackingErrorMetric, IdenticalTrajectoryHasZeroError) {
  Points3d path{ { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 } };
  auto result = TrackingErrorMetric().evaluate(path, path);
  EXPECT_NEAR(result.value, 0.0, 1e-9);
  EXPECT_NEAR(result.details.at("max_error"), 0.0, 1e-9);
}

TEST(TrackingErrorMetric, ConstantPerpendicularOffsetMatchesRmse) {
  Points3d path{ { 0, 0 }, { 10, 0 } };
  const double offset = 0.5;
  Points3d trajectory{ { 0, offset }, { 2, offset }, { 5, offset }, { 8, offset }, { 10, offset } };

  auto result = TrackingErrorMetric().evaluate(path, trajectory);

  EXPECT_NEAR(result.value, offset, 1e-9);
  EXPECT_NEAR(result.details.at("max_error"), offset, 1e-9);
  EXPECT_NEAR(result.details.at("mean_error"), offset, 1e-9);
}

TEST(TrackingErrorMetric, ProjectionClampsToSegmentEnds) {
  // trajectory point sits beyond the end of the (short) reference path;
  // error must be measured to the nearest endpoint, not an infinite line
  Points3d path{ { 0, 0 }, { 1, 0 } };
  Points3d trajectory{ { 5, 0 } };

  auto result = TrackingErrorMetric().evaluate(path, trajectory);

  EXPECT_NEAR(result.value, 4.0, 1e-9);  // distance to (1,0), not projected past it
}

TEST(TrackingErrorMetric, EmptyInputsDoNotCrash) {
  EXPECT_NO_THROW(TrackingErrorMetric().evaluate({}, {}));
  EXPECT_NO_THROW(TrackingErrorMetric().evaluate({ { 0, 0 } }, {}));
  EXPECT_NO_THROW(TrackingErrorMetric().evaluate({}, { { 0, 0 } }));
}

TEST(TrackingErrorMetric, SinglePointPathFallsBackToEuclideanDistance) {
  Points3d path{ { 0, 0 } };
  Points3d trajectory{ { 3, 4 } };

  auto result = TrackingErrorMetric().evaluate(path, trajectory);

  EXPECT_NEAR(result.value, 5.0, 1e-9);  // 3-4-5 triangle
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
