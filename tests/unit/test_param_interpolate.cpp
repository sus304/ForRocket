// ******************************************************
// Unit tests for src/rocket/parameter/interpolate_parameter.cpp
//   - InterpolateParameter wraps a constant, a 1-D linear table, or a 2-D table
//   - exercises every constructor, copy ctor, copy-assignment branch, and
//     both call operators
// ******************************************************

#include <gtest/gtest.h>
#include <vector>
#include <string>

#include "interpolate_parameter.hpp"

using forrocket::InterpolateParameter;

namespace {
// y = 2x + 1 sampled at integer nodes; matches the linear-table reference.
const std::vector<double> kX = {0.0, 1.0, 2.0, 3.0};
const std::vector<double> kY = {1.0, 3.0, 5.0, 7.0};
}  // namespace

// ---------------------------------------------------------------------------
// Decision points covered:
//   InterpolateParameter():            default ctor -> const path, value 0.
//   InterpolateParameter(double):      const-value ctor.
//   InterpolateParameter(x,y,fill):    1-D log ctor (enable_1dlog = true).
//   InterpolateParameter(x,y,z,fill):  2-D log ctor (enable_2dlog = true).
//   InterpolateParameter(const&):      copy ctor, all three branches
//                                      (1dlog / 2dlog / const).
//   operator=(const&):                 self-assign guard + all three branches.
//   operator()(x):                     enable_1dlog true vs false.
//   operator()(x, y):                  always returns 0.0 (stub).
// ---------------------------------------------------------------------------

TEST(InterpolateParam, DefaultConstructorIsConstZero) {
    InterpolateParameter p;
    // Default ctor sets value_const = 0.0, enable_1dlog = false.
    EXPECT_DOUBLE_EQ(p(0.0), 0.0);
    EXPECT_DOUBLE_EQ(p(123.4), 0.0);  // any x returns the constant
}

TEST(InterpolateParam, ConstValueConstructor) {
    InterpolateParameter p(42.0);
    // enable_1dlog == false -> operator()(x) returns value_const for any x.
    EXPECT_DOUBLE_EQ(p(0.0), 42.0);
    EXPECT_DOUBLE_EQ(p(-7.0), 42.0);
    EXPECT_DOUBLE_EQ(p(1e6), 42.0);
}

TEST(InterpolateParam, OneDLinearInterpolation) {
    InterpolateParameter p(kX, kY, "extrapolate");
    // enable_1dlog == true -> delegates to Interp1d on y = 2x + 1.
    EXPECT_DOUBLE_EQ(p(0.0), 1.0);   // node
    EXPECT_DOUBLE_EQ(p(3.0), 7.0);   // node (right edge)
    EXPECT_DOUBLE_EQ(p(1.5), 4.0);   // 3 + 2*0.5
    EXPECT_DOUBLE_EQ(p(2.25), 5.5);  // 5 + 2*0.25
}

TEST(InterpolateParam, OneDFillExtrapolate) {
    InterpolateParameter p(kX, kY, "extrapolate");
    // Outside [0,3] the linear table extrapolates along y = 2x + 1.
    EXPECT_DOUBLE_EQ(p(-1.0), -1.0);  // 1 + 2*(-1)
    EXPECT_DOUBLE_EQ(p(4.0), 9.0);    // 7 + 2*(1)
}

TEST(InterpolateParam, OneDFillZero) {
    InterpolateParameter p(kX, kY, "zero");
    // "zero" fill returns 0 outside the table; interior is unaffected.
    EXPECT_DOUBLE_EQ(p(-1.0), 0.0);
    EXPECT_DOUBLE_EQ(p(4.0), 0.0);
    EXPECT_DOUBLE_EQ(p(1.5), 4.0);
}

TEST(InterpolateParam, TwoDConstructorAndCallReturnsZeroStub) {
    // The 2-D constructor sets enable_2dlog = true but the 2-D polator is not
    // yet implemented; operator()(x, y) is a stub that always returns 0.0.
    std::vector<double> z = {0.0, 1.0, 2.0, 3.0};
    InterpolateParameter p(kX, kY, z, "zero");
    EXPECT_DOUBLE_EQ(p(1.0, 2.0), 0.0);
    EXPECT_DOUBLE_EQ(p(0.5, 0.5), 0.0);
    // For a 2-D-configured parameter, the 1-D call operator (enable_1dlog is
    // false here) falls through to value_const, which is uninitialized for
    // this ctor; we therefore only assert the documented 2-D stub behavior.
}

TEST(InterpolateParam, TwoArgOperatorAlwaysZeroForConst) {
    // operator()(x, y) is a stub returning 0.0 regardless of configuration.
    InterpolateParameter p(42.0);
    EXPECT_DOUBLE_EQ(p(1.0, 2.0), 0.0);
}

TEST(InterpolateParam, CopyConstructorConstBranch) {
    InterpolateParameter src(3.14);
    InterpolateParameter copy(src);  // copy ctor, const branch
    EXPECT_DOUBLE_EQ(copy(0.0), 3.14);
    EXPECT_DOUBLE_EQ(copy(99.0), 3.14);
}

TEST(InterpolateParam, CopyConstructorOneDBranch) {
    InterpolateParameter src(kX, kY, "extrapolate");
    InterpolateParameter copy(src);  // copy ctor, 1dlog branch (deep-copies Interp1d)
    EXPECT_DOUBLE_EQ(copy(1.5), 4.0);   // same y = 2x + 1
    EXPECT_DOUBLE_EQ(copy(0.0), 1.0);
    // Independence: original still works after copy.
    EXPECT_DOUBLE_EQ(src(2.0), 5.0);
}

TEST(InterpolateParam, CopyConstructorTwoDBranch) {
    std::vector<double> z = {0.0, 1.0, 2.0, 3.0};
    InterpolateParameter src(kX, kY, z, "zero");
    InterpolateParameter copy(src);  // copy ctor, 2dlog branch
    EXPECT_DOUBLE_EQ(copy(1.0, 2.0), 0.0);
}

TEST(InterpolateParam, CopyAssignmentConstBranch) {
    InterpolateParameter src(2.5);
    InterpolateParameter dst;
    dst = src;  // operator=, const branch
    EXPECT_DOUBLE_EQ(dst(0.0), 2.5);
    EXPECT_DOUBLE_EQ(dst(10.0), 2.5);
}

TEST(InterpolateParam, CopyAssignmentOneDBranch) {
    InterpolateParameter src(kX, kY, "extrapolate");
    InterpolateParameter dst(7.0);  // start as const, then overwrite
    dst = src;  // operator=, 1dlog branch
    EXPECT_DOUBLE_EQ(dst(1.5), 4.0);
    EXPECT_DOUBLE_EQ(dst(3.0), 7.0);
}

TEST(InterpolateParam, CopyAssignmentTwoDBranch) {
    std::vector<double> z = {0.0, 1.0, 2.0, 3.0};
    InterpolateParameter src(kX, kY, z, "zero");
    InterpolateParameter dst(1.0);
    dst = src;  // operator=, 2dlog branch
    EXPECT_DOUBLE_EQ(dst(1.0, 2.0), 0.0);
}

TEST(InterpolateParam, SelfAssignmentIsSafe) {
    // operator= guards against self-assignment (this != &from).
    InterpolateParameter p(kX, kY, "extrapolate");
    InterpolateParameter& ref = p;
    p = ref;  // self-assign; must remain valid and unchanged
    EXPECT_DOUBLE_EQ(p(1.5), 4.0);
    EXPECT_DOUBLE_EQ(p(0.0), 1.0);
}

TEST(InterpolateParam, AssignmentReturnsLhsReference) {
    // operator= returns *this so chained assignment works.
    InterpolateParameter a(1.0), b(2.0), c(3.0);
    a = b = c;  // c -> b -> a, all become the constant 3.0
    EXPECT_DOUBLE_EQ(a(0.0), 3.0);
    EXPECT_DOUBLE_EQ(b(0.0), 3.0);
}
