#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_HYPOTHESIS_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_HYPOTHESIS_H

#include "contact.h"
#include <vector>
#include <memory>

namespace chair_manipulation
{
struct GraspHypothesis
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspHypothesis() = default;

  GraspHypothesis(const Eigen::Isometry3d& pose, std::vector<Contact> contacts)
    : pose_(pose), contacts_(std::move(contacts))
  {
  }

  /**
   * Pose of tool center point (tcp) where
   *  - the Y-axis is parallel to the gripper pads and points towards the contact point
   *  - the Z-axis is parallel to the gripper and is perpendicular to the Y-axis
   *  - the X-axis is perpendicular to the Y- and X-axis (and therefore perpendicular to the gripper pads)
   */
  Eigen::Isometry3d pose_;

  /**
   * The contacts of the grasp
   */
  std::vector<Contact> contacts_;
};

using GraspHypothesisPtr = std::shared_ptr<GraspHypothesis>;
using GraspHypothesisConstPtr = std::shared_ptr<const GraspHypothesis>;

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_HYPOTHESIS_H
