#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "chair_manipulation_grasp_planning/utils.h"
#include <actionlib/client/simple_action_client.h>
#include <chair_manipulation_msgs/GripperCommandAction.h>

/**
 * This node is only used for testing.
 */

using GripperCommandAction = chair_manipulation_msgs::GripperCommandAction;
using GripperCommandGoal = chair_manipulation_msgs::GripperCommandGoal;
using GripperCommandResult = chair_manipulation_msgs::GripperCommandResult;
using GripperCommandActionClient = actionlib::SimpleActionClient<GripperCommandAction>;

using chair_manipulation::deg2rad;

void moveToPose(moveit::planning_interface::MoveGroupInterface& group,
                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, double x, double y,
                double z, double roll, double pitch, double yaw)
{
  tf2::Quaternion orientation;
  orientation.setRPY(roll, pitch, yaw);
  geometry_msgs::Pose target_pose;
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  group.setPoseReferenceFrame("world");
  group.setPoseTarget(target_pose);

  chair_manipulation::addGroundPlane(planning_scene_interface, group.getPlanningFrame());

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_ERROR_STREAM_NAMED("move", "Failed to plan pose target.");
    return;
  }
  success = group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  if (!success)
  {
    ROS_ERROR_STREAM_NAMED("move", "Failed to execute pose target.");
    return;
  }
}

void moveToNamedTarget(moveit::planning_interface::MoveGroupInterface& group, const std::string& target)
{
  group.setNamedTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_ERROR_STREAM_NAMED("move", "Failed to plan for named target '" << target << "'");
    return;
  }
  success = group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  if (!success)
  {
    ROS_ERROR_STREAM_NAMED("move", "Failed to execute named target '" << target << "'");
    return;
  }
}

void sendGripperCommand(GripperCommandActionClient& client, int goal_id)
{
  GripperCommandGoal goal;
  goal.goal = goal_id;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration{5.});
  if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_ERROR_STREAM_NAMED("move", "Failed to close gripper.");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh_priv{ "~" };

  auto prefix = nh_priv.param<std::string>("prefix", "robot1");
  auto command = nh_priv.param<std::string>("cmd", "move_up");
  auto gripper_command_action_ns = nh_priv.param<std::string>("gripper_command_action_ns", "gripper_command");

  GripperCommandActionClient gripper_command_client{gripper_command_action_ns, true};

  moveit::planning_interface::MoveGroupInterface arm_group(prefix + "_arm");
  moveit::planning_interface::MoveGroupInterface gripper_group(prefix + "_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  arm_group.setPlanningTime(60.0);

  if (command == "move_up")
    moveToNamedTarget(arm_group, prefix + "_up");
  else if (command == "move_home")
    moveToNamedTarget(arm_group, prefix + "_home");
  else if (command == "move_to_pose")
    moveToPose(arm_group, planning_scene_interface, 0.0, -0.5, 0.5, deg2rad(180), deg2rad(0), deg2rad(0));
  else if (command == "close")
    sendGripperCommand(gripper_command_client, GripperCommandGoal::CLOSE);
  else if (command == "open")
    sendGripperCommand(gripper_command_client, GripperCommandGoal::OPEN);
  else
    ROS_WARN("Unknown command!");

  ros::shutdown();
  return 0;
}