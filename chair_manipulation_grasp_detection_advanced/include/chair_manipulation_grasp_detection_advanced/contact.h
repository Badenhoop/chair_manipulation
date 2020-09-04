#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_CONTACT_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_CONTACT_H

#include <Eigen/Dense>

namespace chair_manipulation
{
struct Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Contact() = default;

  Contact(const Eigen::Vector3d& position, const Eigen::Vector3d& normal) : position_(position), normal_(normal)
  {
  }

  /**
   * Position of the contact relative to the world frame.
   */
  Eigen::Vector3d position_;

  /**
   * The normal of the contact relative to the world frame.
   * This always points inside the model.
   */
  Eigen::Vector3d normal_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_CONTACT_H
