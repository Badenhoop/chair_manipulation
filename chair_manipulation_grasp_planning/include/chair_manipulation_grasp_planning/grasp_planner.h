#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/String.h>
#include <mutex>
#include <actionlib/client/simple_action_client.h>
#include <chair_manipulation_msgs/GripperCommandAction.h>

namespace chair_manipulation
{
class GraspPlanner
{
public:
  using GripperCommandAction = chair_manipulation_msgs::GripperCommandAction;
  using GripperCommandGoal = chair_manipulation_msgs::GripperCommandGoal;
  using GripperCommandResult = chair_manipulation_msgs::GripperCommandResult;
  using GripperCommandActionClient = actionlib::SimpleActionClient<GripperCommandAction>;

  explicit GraspPlanner(ros::NodeHandle& nh);

  void prepare();

  void planPreGrasp();

  void executePreGrasp();

  void planGrasp();

  void executeGrasp();

  void planLift();

  void executeLift();

  void stop();

  void cleanup();

private:
  std::string world_frame_;

  std::string robot1_end_effector_frame_;
  std::string robot2_end_effector_frame_;

  std::string robot1_tcp_frame_;
  std::string robot2_tcp_frame_;

  std::string robot1_grasp_frame_;
  std::string robot2_grasp_frame_;

  std::string robot1_planned_pre_grasp_frame_;
  std::string robot2_planned_pre_grasp_frame_;

  std::string robot1_planned_grasp_frame_;
  std::string robot2_planned_grasp_frame_;

  std::string robot1_planned_lift_frame_;
  std::string robot2_planned_lift_frame_;

  std::string object_mesh_topic_;

  int planning_attempts_;
  double planning_attempt_time_;
  double pre_grasp_distance_;
  double lift_height_;
  double max_velocity_scaling_factor_;
  double position_tolerance_;

  std::vector<std::string> touch_links_;

  std::string robot1_gripper_command_action_ns_;
  std::string robot2_gripper_command_action_ns_;

  std::unique_ptr<GripperCommandActionClient> robot1_gripper_command_client_;
  std::unique_ptr<GripperCommandActionClient> robot2_gripper_command_client_;

  std::string arms_group_name_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arms_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  tf2::Transform world_to_robot1_pre_grasp_ee_;
  tf2::Transform world_to_robot2_pre_grasp_ee_;

  tf2::Transform world_to_robot1_grasp_ee_;
  tf2::Transform world_to_robot2_grasp_ee_;

  tf2::Transform world_to_robot1_lift_ee_;
  tf2::Transform world_to_robot2_lift_ee_;

  tf2_ros::StaticTransformBroadcaster broadcaster_;

  shape_msgs::MeshConstPtr object_mesh_;

  void planArmPose(const tf2::Transform& robot1_ee_pose, const tf2::Transform& robot2_ee_pose,
                   const std::string& pose_name);

  void openGripper();

  void closeGripper();

  void setPathConstraints(const tf2::Transform& robot1_goal_pose, const tf2::Transform& robot2_goal_pose);

  void addPathConstraints(const tf2::Transform& goal_pose, const std::string& end_effector_frame,
                          const tf2::Transform& grasp_pose, moveit_msgs::Constraints& constraints);
};

class GraspPlanningException : public std::runtime_error
{
public:
  explicit GraspPlanningException(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
