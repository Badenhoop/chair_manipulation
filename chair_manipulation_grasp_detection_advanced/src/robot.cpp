#include "chair_manipulation_grasp_detection_advanced/robot.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
void RobotParameters::load(ros::NodeHandle& nh)
{
  world_frame_ = nh.param<std::string>("world_frame", "world");
  XmlRpc::XmlRpcValue arm_items;
  if (!nh.getParam("arms", arm_items) || arm_items.getType() != XmlRpc::XmlRpcValue::TypeArray || arm_items.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'arms'." };

  int num_arms = arm_items.size();
  arms_.resize(num_arms);
  for (int i = 0; i < num_arms; i++)
  {
    const auto& item = arm_items[i];
    auto& arm = arms_[i];

    arm.base_frame_ = utils::loadStringParameter(item, "base_frame");
    arm.ik_frame_ = utils::loadStringParameter(item, "ik_frame");
    arm.tcp_frame_ = utils::loadStringParameter(item, "tcp_frame");
    arm.grasp_frame_ = utils::loadStringParameter(item, "grasp_frame");
    arm.group_name_ = utils::loadStringParameter(item, "group_name");
  }
}

RobotArm::RobotArm(RobotArmParameters params, const std::string& world_frame, tf2_ros::Buffer& tf_buffer)
    : params_(std::move(params)), group_(params_.group_name_)
{
  try
  {
    auto msg =
        tf_buffer.lookupTransform(world_frame, params_.base_frame_, ros::Time{ 0 }, ros::Duration{ 3. });
    world_to_base_ = tf2::transformToEigen(msg);

    msg = tf_buffer.lookupTransform(params_.tcp_frame_, params_.ik_frame_, ros::Time{ 0 }, ros::Duration{ 3. });
    tcp_to_ik_ = tf2::transformToEigen(msg);
  }
  catch (const tf2::TransformException& e)
  {
    throw exception::TF{ "Failed to retrieve tf.\nReason:\n " + std::string{ e.what() } };
  }
}

Robot::Robot(RobotParameters params) : params_(std::move(params))
{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  for (const auto& arm_params : params_.arms_)
    arms_.push_back(std::make_shared<RobotArm>(arm_params, params_.world_frame_, tf_buffer));
}

RobotArmPtr Robot::getClosestArm(const Eigen::Isometry3d& pose) const
{
  // Get the arm that is nearest in terms of position
  auto it = std::min_element(arms_.begin(), arms_.end(), [&](const auto& lhs, const auto& rhs) {
    return (pose.translation() - lhs->getBasePose().translation()).norm() <
           (pose.translation() - rhs->getBasePose().translation()).norm();
  });
  return *it;
}

}  // namespace chair_manipulation