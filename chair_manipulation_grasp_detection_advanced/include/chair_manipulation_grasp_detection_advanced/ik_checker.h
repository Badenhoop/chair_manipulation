#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_IK_CHECKER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_IK_CHECKER_H

#include "grasp_hypothesis.h"
#include "robot.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <moveit/move_group_interface/move_group_interface.h>

namespace chair_manipulation
{

struct IKCheckerParameters
{
  void load(ros::NodeHandle& nh);

  double timeout_;
};

class IKChecker
{
public:
  explicit IKChecker(IKCheckerParameters params, const RobotPtr& robot);

  void filter(const std::vector<GraspHypothesis>& hypotheses, std::vector<GraspHypothesis>& filtered_hypotheses);

private:
  IKCheckerParameters params_;
  RobotPtr robot_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_IK_CHECKER_H
