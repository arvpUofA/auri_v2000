#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <angles/angles.h>
#include <eigen3/Eigen/Dense>

namespace au_core {

Eigen::Quaterniond rpyToQuat(const Eigen::Vector3d &rpy);

Eigen::Vector3d quatToRpy(const Eigen::Quaterniond &q);

Eigen::Quaterniond quatDiff(const Eigen::Quaterniond &q1,
                            const Eigen::Quaterniond &q2);

double limitVariable(double v, double min, double max);

double normalizeVariable(double v, double min, double max,
                         double range_min = -1.0, double range_max = 1.0);

double normalizeAngle(double v, double limit);

double radToDeg(double rad);

double degToRad(double deg);

/**
 *       ^ x
 *       |
 *   IV  |   I
 *       |
 * ------------> y
 *       |
 *  III  |  II
 *       |
 *
 * Returns absolute bearing between two points. Bearing ranges
 * from -M_PI to M_PI where 0 is along the positive x-axis and angles
 * increase clockwise.
 */
double bearing(const Eigen::Vector3d &from, const Eigen::Vector3d &to);

}  // namespace au_core

#endif  // PROJECT_MATH_UTIL_H
