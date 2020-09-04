#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MULTI_ARM_GRASP_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MULTI_ARM_GRASP_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <Eigen/StdVector>

namespace chair_manipulation
{
struct MultiArmGrasp
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Pose for each arm with respect to the world frame
   */
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses_;

  /**
   * Overall quality of the grasp
   */
  double quality_;
};

using MultiArmGraspPtr = std::shared_ptr<MultiArmGrasp>;
using MultiArmGraspConstPtr = std::shared_ptr<const MultiArmGrasp>;

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MULTI_ARM_GRASP_H
