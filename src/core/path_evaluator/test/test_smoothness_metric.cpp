#include <cmath>

#include <gtest/gtest.h>

#include "path_evaluator/metrics/smoothness_metric.h"

using rmp::common::geometry::Points3d;
using rmp::path_evaluator::SmoothnessMetric;

TEST(SmoothnessMetric, StraightLineHasZeroCurvature) {
  Points3d path{ { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 } };
  auto result = SmoothnessMetric().evaluate(path);
  EXPECT_NEAR(result.value, 0.0, 1e-9);
  EXPECT_NEAR(result.details.at("max_curvature"), 0.0, 1e-9);
}

TEST(SmoothnessMetric, RightAngleCornerSpikesCurvature) {
  Points3d path{ { 0, 0 }, { 1, 0 }, { 2, 0 }, { 2, 1 }, { 2, 2 } };
  auto result = SmoothnessMetric().evaluate(path);
  EXPECT_GT(result.details.at("max_curvature"), 1.0);
}

namespace {
// Points on a quarter-circle of radius R, sampled at N equal angular steps.
// Curvature of a true circular arc is constant (kappa = 1/R) regardless of
// how many points sample it -- unlike a sharp kink, whose curvature is a
// discontinuity and legitimately reads differently depending on how close
// the two neighboring samples are to the corner.
Points3d quarterCircle(double radius, int n) {
  Points3d path;
  for (int i = 0; i <= n; ++i) {
    double theta = (M_PI / 2.0) * i / n;
    path.emplace_back(radius * std::sin(theta), radius * (1 - std::cos(theta)));
  }
  return path;
}
}  // namespace

TEST(SmoothnessMetric, SamplingDensityInvariantOnSmoothCurve) {
  const double radius = 5.0;
  auto coarse = SmoothnessMetric().evaluate(quarterCircle(radius, 4));
  auto fine = SmoothnessMetric().evaluate(quarterCircle(radius, 16));

  // both should read close to the true curvature 1/R, independent of density
  EXPECT_NEAR(coarse.value, 1.0 / radius, 0.05);
  EXPECT_NEAR(fine.value, 1.0 / radius, 0.01);
}

TEST(SmoothnessMetric, ShortPathDoesNotCrash) {
  EXPECT_NO_THROW(SmoothnessMetric().evaluate({}));
  EXPECT_NO_THROW(SmoothnessMetric().evaluate({ { 0, 0 } }));
  EXPECT_NO_THROW(SmoothnessMetric().evaluate({ { 0, 0 }, { 1, 0 } }));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
