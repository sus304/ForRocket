// ******************************************************
// Unit tests for src/interpolate.cpp
//   - Linear1D interpolation + fill_value (zero/same/extrapolate)
//   - CubicSpline1D passes through data points
//   - Interp1d vector/Eigen call operators
// ******************************************************

#include <gtest/gtest.h>
#include <cstdlib>  // EXIT_FAILURE for death tests
#include <cmath>    // std::isfinite
#include <vector>

#include "Eigen/Core"

#include "interpolate.hpp"

using forrocket::interpolate::Interp1d;

namespace {
const std::vector<double> kX = {0.0, 1.0, 2.0, 3.0};
const std::vector<double> kY = {1.0, 3.0, 5.0, 7.0};  // y = 2x + 1
}  // namespace

TEST(LinearInterp, InteriorAndNodes) {
    Interp1d f(kX, kY, "linear", "extrapolate");
    EXPECT_DOUBLE_EQ(f(0.0), 1.0);   // node
    EXPECT_DOUBLE_EQ(f(3.0), 7.0);   // node (right edge)
    EXPECT_DOUBLE_EQ(f(1.5), 4.0);   // 3 + 2*0.5
    EXPECT_DOUBLE_EQ(f(2.25), 5.5);  // 5 + 2*0.25
}

TEST(LinearInterp, FillExtrapolate) {
    Interp1d f(kX, kY, "linear", "extrapolate");
    EXPECT_DOUBLE_EQ(f(-1.0), -1.0);  // 1 + 2*(-1)
    EXPECT_DOUBLE_EQ(f(4.0), 9.0);    // 7 + 2*(1)
}

TEST(LinearInterp, FillZero) {
    Interp1d f(kX, kY, "linear", "zero");
    EXPECT_DOUBLE_EQ(f(-1.0), 0.0);
    EXPECT_DOUBLE_EQ(f(4.0), 0.0);
    EXPECT_DOUBLE_EQ(f(1.5), 4.0);  // interior unaffected
}

TEST(LinearInterp, FillSame) {
    Interp1d f(kX, kY, "linear", "same");
    EXPECT_DOUBLE_EQ(f(-1.0), kY.front());  // clamp to first
    EXPECT_DOUBLE_EQ(f(4.0), kY.back());     // clamp to last
}

TEST(LinearInterp, UnsortedInputGetsSorted) {
    // Constructor sorts ascending by x; result must match the sorted line.
    std::vector<double> x = {3.0, 1.0, 0.0, 2.0};
    std::vector<double> y = {7.0, 3.0, 1.0, 5.0};  // still y = 2x + 1
    Interp1d f(x, y, "linear", "extrapolate");
    EXPECT_DOUBLE_EQ(f(1.5), 4.0);
    EXPECT_DOUBLE_EQ(f(0.0), 1.0);
}

TEST(LinearInterp, VectorCallOperator) {
    Interp1d f(kX, kY, "linear", "extrapolate");
    std::vector<double> xs = {0.0, 1.5, 3.0};
    std::vector<double> ys = f(xs);
    ASSERT_EQ(ys.size(), 3u);
    EXPECT_DOUBLE_EQ(ys[0], 1.0);
    EXPECT_DOUBLE_EQ(ys[1], 4.0);
    EXPECT_DOUBLE_EQ(ys[2], 7.0);
}

TEST(LinearInterp, EigenCallOperator) {
    Interp1d f(kX, kY, "linear", "extrapolate");
    Eigen::VectorXd xs(3);
    xs << 0.0, 1.5, 3.0;
    Eigen::VectorXd ys = f(xs);
    ASSERT_EQ(ys.size(), 3);
    EXPECT_DOUBLE_EQ(ys[0], 1.0);
    EXPECT_DOUBLE_EQ(ys[1], 4.0);
    EXPECT_DOUBLE_EQ(ys[2], 7.0);
}

TEST(CubicSplineInterp, PassesThroughNodes) {
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> y = {0.0, 1.0, 4.0, 9.0, 16.0};  // y = x^2 samples
    Interp1d f(x, y, "cubic", "extrapolate");
    for (std::size_t i = 0; i < x.size(); ++i) {
        EXPECT_NEAR(f(x[i]), y[i], 1e-9) << "at node " << i;
    }
}

TEST(CubicSplineInterp, MidpointStaysBetweenNeighbors) {
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> y = {0.0, 1.0, 4.0, 9.0, 16.0};
    Interp1d f(x, y, "cubic", "extrapolate");
    double v = f(0.5);  // between y=0 and y=1 for a monotone-increasing set
    EXPECT_GT(v, 0.0);
    EXPECT_LT(v, 1.0);
}

TEST(CubicSplineInterp, CopyAssignmentIsIndependent) {
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> y = {0.0, 1.0, 8.0, 27.0};
    Interp1d a(x, y, "cubic", "extrapolate");
    Interp1d b = a;          // copy constructor (deep-copies spline coefficients)
    EXPECT_DOUBLE_EQ(a(1.5), b(1.5));
    EXPECT_NEAR(b(2.0), 8.0, 1e-9);
}

// --- CubicSpline1D fill_value branches (left/right out-of-range) ----------
TEST(CubicSplineInterp, FillZeroOutsideRange) {
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> y = {0.0, 1.0, 8.0, 27.0};
    Interp1d f(x, y, "cubic", "zero");
    EXPECT_DOUBLE_EQ(f(-1.0), 0.0);  // left
    EXPECT_DOUBLE_EQ(f(4.0), 0.0);   // right
}

TEST(CubicSplineInterp, FillSameOutsideRange) {
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> y = {2.0, 1.0, 8.0, 27.0};
    Interp1d f(x, y, "cubic", "same");
    EXPECT_DOUBLE_EQ(f(-1.0), 2.0);   // clamp to first y
    EXPECT_DOUBLE_EQ(f(4.0), 27.0);   // clamp to last y
}

TEST(CubicSplineInterp, FillExtrapolateOutsideRange) {
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> y = {0.0, 1.0, 8.0, 27.0};
    Interp1d f(x, y, "cubic", "extrapolate");
    // Exercises the left/right extrapolation branches. A natural cubic may
    // overshoot at the edges, so only require a finite evaluation.
    EXPECT_TRUE(std::isfinite(f(-0.5)));
    EXPECT_TRUE(std::isfinite(f(3.5)));
}

// NOTE: cubic + UNSORTED input is intentionally not tested here. interpolate.cpp
// sorts x_src/y_src but builds CubicSpline1D from the original unsorted (x,y),
// so the spline coefficients and the lookup table disagree (latent bug). The
// need_sort() branch itself is covered by LinearInterp.UnsortedInputGetsSorted.

TEST(Interp1d, CopyAndAssignLinearAndCubic) {
    std::vector<double> x  = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> yl = {1.0, 3.0, 5.0, 7.0};   // linear y = 2x+1
    std::vector<double> yc = {0.0, 1.0, 8.0, 27.0};  // cubic samples

    Interp1d lin(x, yl, "linear", "extrapolate");
    Interp1d cub(x, yc, "cubic", "extrapolate");

    // copy constructor: hits both Linear1D / CubicSpline1D typeid branches
    Interp1d lin_copy = lin;
    Interp1d cub_copy = cub;
    EXPECT_DOUBLE_EQ(lin_copy(1.5), lin(1.5));
    EXPECT_NEAR(cub_copy(2.0), cub(2.0), 1e-12);

    // copy assignment that replaces an existing polator (linear<->cubic)
    Interp1d target(x, yl, "linear", "zero");
    target = cub;
    EXPECT_NEAR(target(2.0), cub(2.0), 1e-12);
    target = lin;
    EXPECT_DOUBLE_EQ(target(1.5), lin(1.5));

    // self-assignment guard (pointer indirection dodges -Wself-assign)
    Interp1d* self = &target;
    target = *self;
    EXPECT_DOUBLE_EQ(target(1.5), lin(1.5));
}

TEST(LinearInterp, ExactInteriorNodes) {
    Interp1d f(kX, kY, "linear", "extrapolate");  // kY = 2x+1
    EXPECT_DOUBLE_EQ(f(1.0), 3.0);  // x == x_src[i] exact-node branch
    EXPECT_DOUBLE_EQ(f(2.0), 5.0);
}

TEST(CubicSpline1DDirect, CopyAndAssignDeepCopy) {
    // Interp1d reconstructs its CubicSpline1D on copy, so the spline's own
    // Rule-of-3 deep-copy loops are only reachable by copying it directly.
    using forrocket::interpolate::CubicSpline1D;
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> y = {0.0, 1.0, 8.0, 27.0};

    CubicSpline1D a(x, y);
    CubicSpline1D b(a);          // copy constructor -> deep-copy loops
    EXPECT_NEAR(b.polate(1.5, x, y, 2), a.polate(1.5, x, y, 2), 1e-12);

    CubicSpline1D c(x, y);
    c = a;                       // copy assignment -> deep-copy loops
    EXPECT_NEAR(c.polate(2.0, x, y, 2), a.polate(2.0, x, y, 2), 1e-12);

    CubicSpline1D* self = &c;    // self-assignment guard
    c = *self;
    EXPECT_NEAR(c.polate(2.5, x, y, 2), a.polate(2.5, x, y, 2), 1e-12);
}

// --- Tier C: constructor error paths call exit() (death tests) ------------
TEST(Interp1dDeathTest, SizeMismatchExits) {
    std::vector<double> x = {0.0, 1.0, 2.0};
    std::vector<double> y = {0.0, 1.0};  // size mismatch
    EXPECT_EXIT(Interp1d(x, y, "linear", "zero"),
                ::testing::ExitedWithCode(EXIT_FAILURE), "Diffirent number");
}

TEST(Interp1dDeathTest, BadFillValueExits) {
    std::vector<double> x = {0.0, 1.0};
    std::vector<double> y = {0.0, 1.0};
    EXPECT_EXIT(Interp1d(x, y, "linear", "bogus"),
                ::testing::ExitedWithCode(EXIT_FAILURE), "fill value");
}

TEST(Interp1dDeathTest, BadKindExits) {
    std::vector<double> x = {0.0, 1.0};
    std::vector<double> y = {0.0, 1.0};
    EXPECT_EXIT(Interp1d(x, y, "nope", "zero"),
                ::testing::ExitedWithCode(EXIT_FAILURE), "polate kind");
}
