// ******************************************************
// Unit tests for src/environment/coordinate.cpp
//   - LLH <-> ECEF geodetic conversion
//   - NED2Body direction cosine matrix
//   - Euler <-> Quaternion round trip
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "Eigen/Dense"  // determinant() lives in the LU module

#include "environment/coordinate.hpp"
#include "environment/wgs84.hpp"
#include "degrad.hpp"

using forrocket::Coordinate;
using forrocket::WGS84;
using forrocket::deg2rad;

TEST(Coordinate, LLH2ECEF_Equator) {
    // lat=0, lon=0, h=0  ->  x = semi-major axis, y = z = 0
    Coordinate coord;
    Eigen::Vector3d ecef = coord.LLH2ECEF(Eigen::Vector3d(0.0, 0.0, 0.0));
    WGS84 wgs;
    EXPECT_NEAR(ecef(0), wgs.a, 1e-6);
    EXPECT_NEAR(ecef(1), 0.0, 1e-6);
    EXPECT_NEAR(ecef(2), 0.0, 1e-6);
}

TEST(Coordinate, LLH2ECEF_NorthPole) {
    // lat=90, h=0  ->  z = semi-minor axis, x = y = 0
    Coordinate coord;
    Eigen::Vector3d ecef = coord.LLH2ECEF(Eigen::Vector3d(90.0, 0.0, 0.0));
    WGS84 wgs;
    EXPECT_NEAR(ecef(0), 0.0, 1e-6);
    EXPECT_NEAR(ecef(1), 0.0, 1e-6);
    EXPECT_NEAR(ecef(2), wgs.b, 1e-6);
}

TEST(Coordinate, ECEF2LLH_Equator) {
    Coordinate coord;
    WGS84 wgs;
    Eigen::Vector3d llh = coord.ECEF2LLH(Eigen::Vector3d(wgs.a, 0.0, 0.0));
    EXPECT_NEAR(llh(0), 0.0, 1e-9);  // lat [deg]
    EXPECT_NEAR(llh(1), 0.0, 1e-9);  // lon [deg]
    EXPECT_NEAR(llh(2), 0.0, 1e-6);  // height [m]
}

TEST(Coordinate, LLH_ECEF_RoundTrip) {
    Coordinate coord;
    // A few terrestrial points (lat[deg], lon[deg], height[m]).
    const Eigen::Vector3d points[] = {
        Eigen::Vector3d(35.0, 139.0, 1000.0),    // Tokyo-ish
        Eigen::Vector3d(-33.9, 151.2, 50.0),     // Sydney-ish
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(60.0, -120.0, 8000.0),
    };
    for (const auto& llh : points) {
        Eigen::Vector3d back = coord.ECEF2LLH(coord.LLH2ECEF(llh));
        EXPECT_NEAR(back(0), llh(0), 1e-7) << "lat";
        EXPECT_NEAR(back(1), llh(1), 1e-7) << "lon";
        EXPECT_NEAR(back(2), llh(2), 1e-4) << "height";
    }
}

TEST(Coordinate, NED2Body_ZeroAttitudeIsIdentity) {
    Coordinate coord;
    coord.setNED2Body(Eigen::Vector3d(0.0, 0.0, 0.0));  // azimuth, elevation, roll
    Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(coord.dcm.NED2body.isApprox(expected, 1e-12))
        << coord.dcm.NED2body;
    // body2NED is the transpose of NED2body.
    EXPECT_TRUE(coord.dcm.body2NED.isApprox(coord.dcm.NED2body.transpose(), 1e-15));
}

TEST(Coordinate, NED2Body_IsOrthonormal) {
    Coordinate coord;
    coord.setNED2Body(Eigen::Vector3d(deg2rad(30.0), deg2rad(10.0), deg2rad(5.0)));
    Eigen::Matrix3d C = coord.dcm.NED2body;
    // A valid rotation matrix: C * C^T = I and det(C) = +1.
    EXPECT_TRUE((C * C.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(C.determinant(), 1.0, 1e-12);
}

TEST(Coordinate, EulerQuaternionRoundTrip) {
    Coordinate coord;
    Eigen::Vector3d euler(deg2rad(30.0), deg2rad(10.0), deg2rad(5.0));  // azi, elv, rol

    Eigen::Vector4d quat = coord.Quaternion(euler);
    EXPECT_NEAR(quat.norm(), 1.0, 1e-12);  // unit quaternion

    coord.setNED2Body(quat);            // rebuild DCM from the quaternion
    Eigen::Vector3d recovered = coord.EulerAngle();
    EXPECT_NEAR(recovered(0), euler(0), 1e-9) << "azimuth";
    EXPECT_NEAR(recovered(1), euler(1), 1e-9) << "elevation";
    EXPECT_NEAR(recovered(2), euler(2), 1e-9) << "roll";
}

TEST(Coordinate, QuaternionSwitchAllBranches) {
    // Quaternion() picks the largest component for numerical stability via a
    // 4-way switch. Drive every case with an attitude that makes a different
    // component dominant. Invariant valid for ALL cases: the quaternion must
    // reproduce the same NED2body DCM that the Euler angles produced.
    struct Case { double azi, elv, rol; const char* name; };
    const Case cases[] = {
        {0.0,            0.0, 0.0,             "scalar q3 (identity)"},
        {0.0,            0.0, deg2rad(179.0),  "vector x q0 (roll~180)"},
        {deg2rad(179.0), 0.0, deg2rad(179.0),  "vector y q1 (yaw+roll~180)"},
        {deg2rad(179.0), 0.0, 0.0,             "vector z q2 (yaw~180)"},
    };
    for (const auto& c : cases) {
        Coordinate coord;
        Eigen::Vector3d euler(c.azi, c.elv, c.rol);
        coord.setNED2Body(euler);
        Eigen::Matrix3d from_euler = coord.dcm.NED2body;

        Eigen::Vector4d q = coord.Quaternion(euler);
        EXPECT_NEAR(q.norm(), 1.0, 1e-12) << c.name;

        coord.setNED2Body(q);  // rebuild the DCM from the quaternion
        EXPECT_TRUE(coord.dcm.NED2body.isApprox(from_euler, 1e-9)) << c.name;
    }
}
