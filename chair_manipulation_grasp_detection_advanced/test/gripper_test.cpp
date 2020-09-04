#include "test_common.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/gripper.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gripper_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };
  auto mesh_pub = nh.advertise<shape_msgs::Mesh>("mesh", 1);
  auto robot_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);
  tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped msg;
  TestParameters params;

  auto tcp_pose_str = nh_priv.param<std::string>("tcp_pose", "0.314355 0.015 0.575235 0 -0.707107 0 0.707107");
  Eigen::Isometry3d tcp_pose = utils::poseFromStr(tcp_pose_str);

  auto gripper = std::make_shared<Gripper>(params.gripper_params_, params.gripper_urdf_, params.gripper_srdf_);
  gripper->setTcpPose(tcp_pose);
  gripper->setStateOpen();
  gripper->addCollisionObject(params.model_->getMesh());

  bool colliding = gripper->isColliding();
  ROS_INFO_STREAM("colliding: " << colliding);

  std::vector<Contact> contacts;
  bool success = gripper->grasp(contacts);
  ROS_INFO_STREAM("grasp success: " << success);

  msg = tf2::eigenToTransform(gripper->getBasePose());
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "parent_world";
  msg.child_frame_id = "world";
  broadcaster.sendTransform(msg);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("parent_world");
  visual_tools.deleteAllMarkers();

  for (const auto& contact : contacts)
    visual_tools.publishArrow(Eigen::Translation3d{ contact.position_ } * transform::fromXAxis(contact.normal_));

  visual_tools.trigger();

  shape_msgs::Mesh mesh_msg;
  chair_manipulation::utils::shapeMeshToMsg(*params.model_->getMesh(), mesh_msg);

  auto robot_state = gripper->getRobotState();
  moveit_msgs::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg.state);
  robot_state_msg.hide = false;

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    mesh_pub.publish(mesh_msg);
    robot_state_pub.publish(robot_state_msg);
    rate.sleep();
  }

  return 0;
}