#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_ROBOT_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_ROBOT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>

namespace chair_manipulation
{
struct RobotArmParameters
{
  std::string base_frame_;
  std::string ik_frame_;
  std::string tcp_frame_;
  std::string grasp_frame_;
  std::string group_name_;
};

struct RobotParameters
{
  void load(ros::NodeHandle& nh);

  std::string world_frame_;
  std::vector<RobotArmParameters> arms_;
};

class RobotArm
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit RobotArm(RobotArmParameters params, const std::string& world_frame, tf2_ros::Buffer& tf_buffer);

  std::string getBaseFrame() const
  {
    return params_.base_frame_;
  }

  std::string getIkFrame() const
  {
    return params_.ik_frame_;
  }

  std::string getTcpFrame() const
  {
    return params_.tcp_frame_;
  }

  std::string getGraspFrame() const
  {
    return params_.grasp_frame_;
  }

  Eigen::Isometry3d getBasePose() const
  {
    return world_to_base_;
  }

  Eigen::Isometry3d getTcpToIk() const
  {
    return tcp_to_ik_;
  }

  moveit::planning_interface::MoveGroupInterface* getMoveGroup()
  {
    return &group_;
  }

private:
  RobotArmParameters params_;
  Eigen::Isometry3d world_to_base_;
  Eigen::Isometry3d tcp_to_ik_;
  moveit::planning_interface::MoveGroupInterface group_;
};

using RobotArmPtr = std::shared_ptr<RobotArm>;
using RobotArmConstPtr = std::shared_ptr<const RobotArm>;

class Robot
{
public:
  explicit Robot(RobotParameters params);

  std::string getWorldFrame() const
  {
    return params_.world_frame_;
  }

  RobotArmPtr getClosestArm(const Eigen::Isometry3d& pose) const;

private:
  RobotParameters params_;
  std::vector<RobotArmPtr> arms_;
};

using RobotPtr = std::shared_ptr<Robot>;
using RobotConstPtr = std::shared_ptr<const Robot>;

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_ROBOT_H
