#include <au_core/math_util.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

Eigen::Quaterniond au_core::rpyToQuat(const Eigen::Vector3d &rpy) {
  // YPR - ZYX
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
}

Eigen::Vector3d au_core::quatToRpy(const Eigen::Quaterniond &q) {
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

Eigen::Quaterniond au_core::quatDiff(const Eigen::Quaterniond &q1,
                                     const Eigen::Quaterniond &q2) {
  return q2 * q1.inverse();
}

double au_core::limitVariable(double v, double min, double max) {
  double out = v < min ? min : v;
  return out > max ? max : out;
}

double au_core::normalizeVariable(double v, double min, double max,
                                  double range_min, double range_max) {
  return (range_max - range_min) * (v - min) / (max - min) + range_min;
}

double au_core::radToDeg(double rad) { return rad * 180.0 / M_PI; }

double au_core::degToRad(double deg) { return deg * M_PI / 180.0; }

double au_core::normalizeAngle(double v, double limit) {
  return std::fmod(v + limit, limit * 2) - limit;
}

double au_core::bearing(const Eigen::Vector3d &from,
                        const Eigen::Vector3d &to) {
  return normalizeAngle(atan2(to.y() - from.y(), to.x() - from.x()), M_PI);
}
