#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_TRANSFORM_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_TRANSFORM_H

#include <Eigen/Dense>

namespace chair_manipulation
{
namespace transform
{
Eigen::Isometry3d fromXAxis(const Eigen::Vector3d& x_axis);

Eigen::Isometry3d fromYAxis(const Eigen::Vector3d& y_axis);

Eigen::Isometry3d fromZAxis(const Eigen::Vector3d& z_axis);

Eigen::Isometry3d fromXYAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis);

Eigen::Isometry3d fromYXAxes(const Eigen::Vector3d& y_axis, const Eigen::Vector3d& x_axis);

Eigen::Isometry3d fromXZAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& z_axis);

Eigen::Isometry3d fromZXAxes(const Eigen::Vector3d& z_axis, const Eigen::Vector3d& x_axis);

Eigen::Isometry3d fromYZAxes(const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis);

Eigen::Isometry3d fromZYAxes(const Eigen::Vector3d& z_axis, const Eigen::Vector3d& y_axis);

Eigen::Isometry3d fromXYZAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis,
                              const Eigen::Vector3d& z_axis);

void perpendicularAxes(const Eigen::Vector3d& axis1, Eigen::Vector3d& axis2, Eigen::Vector3d& axis3);

}  // namespace transform
}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_TRANSFORM_H
