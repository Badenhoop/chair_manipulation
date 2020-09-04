#include "chair_manipulation_grasp_detection_advanced/transform.h"

namespace chair_manipulation
{
namespace transform
{
constexpr double EPSILON = 1.0e-10;

void perpendicularAxis(const Eigen::Vector3d& primary_axis, const Eigen::Vector3d& secondary_axis,
                       Eigen::Vector3d& new_secondary_axis, Eigen::Vector3d& perpendicular_axis)
{
  perpendicular_axis = primary_axis.cross(secondary_axis);
  new_secondary_axis = perpendicular_axis.cross(primary_axis);
}

void perpendicularAxes(const Eigen::Vector3d& axis1, Eigen::Vector3d& axis2, Eigen::Vector3d& axis3)
{
  if (std::abs(axis1.dot(Eigen::Vector3d::UnitX())) < 1. - EPSILON)
    axis2 = axis1.cross(Eigen::Vector3d::UnitX());
  else
    axis2 = axis1.cross(Eigen::Vector3d::UnitY());

  axis3 = axis1.cross(axis2);
}

Eigen::Isometry3d fromXAxis(const Eigen::Vector3d& x_axis)
{
  Eigen::Vector3d axis2, axis3;
  perpendicularAxes(x_axis, axis2, axis3);
  return fromXYZAxes(x_axis, axis2, axis3);
}

Eigen::Isometry3d fromYAxis(const Eigen::Vector3d& y_axis)
{
  Eigen::Vector3d axis2, axis3;
  perpendicularAxes(y_axis, axis2, axis3);
  return fromXYZAxes(axis2, y_axis, axis3);
}

Eigen::Isometry3d fromZAxis(const Eigen::Vector3d& z_axis)
{
  Eigen::Vector3d axis2, axis3;
  perpendicularAxes(z_axis, axis2, axis3);
  return fromXYZAxes(axis2, axis3, z_axis);
}

Eigen::Isometry3d fromXYAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis)
{
  Eigen::Vector3d new_y_axis, new_z_axis;
  perpendicularAxis(x_axis, y_axis, new_y_axis, new_z_axis);
  return fromXYZAxes(x_axis, new_y_axis, new_z_axis);
}

Eigen::Isometry3d fromYXAxes(const Eigen::Vector3d& y_axis, const Eigen::Vector3d& x_axis)
{
  Eigen::Vector3d new_x_axis, new_z_axis;
  perpendicularAxis(y_axis, x_axis, new_x_axis, new_z_axis);
  return fromXYZAxes(new_x_axis, y_axis, new_z_axis);
}

Eigen::Isometry3d fromXZAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& z_axis)
{
  Eigen::Vector3d new_z_axis, new_y_axis;
  perpendicularAxis(x_axis, z_axis, new_z_axis, new_y_axis);
  return fromXYZAxes(x_axis, new_y_axis, new_z_axis);
}

Eigen::Isometry3d fromZXAxes(const Eigen::Vector3d& z_axis, const Eigen::Vector3d& x_axis)
{
  Eigen::Vector3d new_x_axis, new_y_axis;
  perpendicularAxis(z_axis, x_axis, new_x_axis, new_y_axis);
  return fromXYZAxes(new_x_axis, new_y_axis, z_axis);
}

Eigen::Isometry3d fromYZAxes(const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis)
{
  Eigen::Vector3d new_z_axis, new_x_axis;
  perpendicularAxis(y_axis, z_axis, new_z_axis, new_x_axis);
  return fromXYZAxes(new_x_axis, y_axis, new_z_axis);
}

Eigen::Isometry3d fromXYZAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis,
                              const Eigen::Vector3d& z_axis)
{
  Eigen::Matrix3d R;
  R.col(0) = x_axis.normalized();
  R.col(1) = y_axis.normalized();
  R.col(2) = z_axis.normalized();
  Eigen::Quaterniond q;
  q = R;
  q.normalize();
  Eigen::Isometry3d T;
  T = q;
  return T;
}

}  // namespace transform
}  // namespace chair_manipulation
