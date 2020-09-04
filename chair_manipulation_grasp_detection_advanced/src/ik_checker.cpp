#include "chair_manipulation_grasp_detection_advanced/ik_checker.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"

namespace chair_manipulation
{
void IKCheckerParameters::load(ros::NodeHandle& nh)
{
  timeout_ = nh.param<double>("timeout", 0.1);
}

IKChecker::IKChecker(IKCheckerParameters params, const RobotPtr& robot) : params_(std::move(params)), robot_(robot)
{
}

void IKChecker::filter(const std::vector<GraspHypothesis>& hypotheses,
                       std::vector<GraspHypothesis>& filtered_hypotheses)
{
  for (const auto& hypothesis : hypotheses)
  {
    const auto& pose = hypothesis.pose_;
    auto arm = robot_->getClosestArm(pose);

    Eigen::Isometry3d world_to_ik = pose * arm->getTcpToIk();
    auto group = arm->getMoveGroup();
    auto state = *group->getCurrentState();
    bool success =
        state.setFromIK(state.getJointModelGroup(group->getName()), world_to_ik, arm->getIkFrame(), params_.timeout_);
    if (!success)
    {
      ROS_DEBUG_STREAM_NAMED("ik_checker", "Unable to solve IK for pose [" << utils::poseToStr(pose) << "]");
      continue;
    }
    filtered_hypotheses.push_back(hypothesis);
  }
  ROS_DEBUG_STREAM_NAMED("ik_checker",
                         "Kept " << filtered_hypotheses.size() << "/" << hypotheses.size() << " grasp hypotheses.");
}

}  // namespace chair_manipulation
