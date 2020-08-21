#include <au_core/math_util.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <vector>

using namespace au_core;

// generated using https://www.andre-gaschler.com/rotationconverter/
// ZYX Euler conversion

std::vector<Eigen::Vector3d> rpy_test_data = {Eigen::Vector3d(1.0, 1.0, 1.0),
                                              Eigen::Vector3d(0.2, 0.5, 0.9),
                                              Eigen::Vector3d(1.5, 1.5, 0.0)};

std::vector<Eigen::Quaterniond> quat_test_data = {
    Eigen::Quaterniond(0.7860666, 0.1675188, 0.5709415, 0.1675188),
    Eigen::Quaterniond(0.878839, -0.0199745, 0.2637354, 0.3970977),
    Eigen::Quaterniond(0.5353686, 0.4987475, 0.4987475, -0.4646314)};

void EXPECT_QUATERNION_EQ(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
  EXPECT_NEAR(std::abs(q1.dot(q2)), 1, 0.02);
}

void EXPECT_VECTOR3D_EQ(Eigen::Vector3d v1, Eigen::Vector3d v2) {
  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(v1[i], v2[i], 0.01);
  }
}

TEST(MathUtilTest, convertRpyToQuaternion) {
  for (size_t i = 0; i < rpy_test_data.size(); i++) {
    Eigen::Quaterniond q = rpyToQuat(rpy_test_data[i]);
    EXPECT_QUATERNION_EQ(q, quat_test_data[i]);
  }
}

TEST(MathUtilTest, convertQuaternionToRpy) {
  for (size_t i = 0; i < quat_test_data.size(); i++) {
    Eigen::Vector3d rpy = quatToRpy(quat_test_data[i]);
    EXPECT_VECTOR3D_EQ(rpy, rpy_test_data[i]);
  }
}

TEST(MathUtilTest, quatError) {
  Eigen::Quaterniond q1(-0.914, -0.001, -0.386, 0.129);
  EXPECT_QUATERNION_EQ(quatDiff(q1, q1), Eigen::Quaterniond::Identity());
}

TEST(MathUtilTest, variableLimit) {
  EXPECT_DOUBLE_EQ(limitVariable(69, -100, 100), 69);
  EXPECT_DOUBLE_EQ(limitVariable(-69, -100, 100), -69);
  EXPECT_DOUBLE_EQ(limitVariable(-169, -100, 100), -100);
  EXPECT_DOUBLE_EQ(limitVariable(169, -100, 100), 100);
}

TEST(MathUtilTest, normalizeVariable) {
  EXPECT_DOUBLE_EQ(normalizeVariable(0, -M_PI, M_PI), 0);
  EXPECT_DOUBLE_EQ(normalizeVariable(M_PI, -M_PI, M_PI), 1);
  EXPECT_DOUBLE_EQ(normalizeVariable(-M_PI, -M_PI, M_PI), -1);
  EXPECT_DOUBLE_EQ(normalizeVariable(0, 0, 12, 0, 100), 0);
  EXPECT_DOUBLE_EQ(normalizeVariable(6, 0, 12, 0, 100), 50);
  EXPECT_DOUBLE_EQ(normalizeVariable(-M_PI_2, -M_PI, M_PI, -100, 100), -50);
}

Eigen::Vector3d point_xy(double x, double y) {
  return Eigen::Vector3d{x, y, 0};
}

void one_bearing_test(double q1_expected, const Eigen::Vector3d& q1_vector) {
  static const Eigen::Quaterniond rot_90 =
      rpyToQuat(Eigen::Vector3d{0, 0, M_PI_2});
  static const Eigen::Vector3d origin = point_xy(0, 0);

  // Preconditions
  EXPECT_TRUE((0 <= q1_expected) && (q1_expected <= M_PI_2))
      << "q1_expected should be in first quadrant and in radians";

  // Tests all quadrants
  /**
   *       ^ x
   *       |
   *   IV  |   I
   *       |
   * ------------> y
   *       |
   *  III  |  II
   *       |
   */

  Eigen::Vector3d to = q1_vector;
  for (int quadrant = 1; quadrant <= 4; quadrant++) {
    double expected =
        normalizeAngle(q1_expected + (quadrant - 1) * M_PI_2, M_PI);
    EXPECT_NEAR(expected, bearing(origin, to), 0.01)
        << "Incorrect bearing in quadrant " << quadrant;
    to = rot_90 * to;
  }
}

TEST(MathUtilTest, bearing) {
  one_bearing_test(0, point_xy(1, 0));
  one_bearing_test(degToRad(15), point_xy(0.9659, 0.2588));
  one_bearing_test(degToRad(30), point_xy(0.8660, 0.5));
  one_bearing_test(degToRad(45), point_xy(12, 12));
  one_bearing_test(degToRad(60), point_xy(0.5, 0.8660));
  one_bearing_test(degToRad(82), point_xy(0.1392, 0.9902));
  one_bearing_test(degToRad(13.855), point_xy(1.7321, 0.4272));

  // A "raw" test for different origins
  EXPECT_NEAR(degToRad(128.1572), bearing(point_xy(16, -4), point_xy(5, 10)),
              0.01);
}

// Run all the tests
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
