#include "chair_manipulation_grasp_planning/grasp_planner.h"
#include "chair_manipulation_grasp_planning/utils.h"
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <thread>

namespace chair_manipulation
{
GraspPlanner::GraspPlanner(ros::NodeHandle& nh)
{
  world_frame_ = nh.param<std::string>("world_frame", "world");

  robot1_end_effector_frame_ = nh.param<std::string>("robot1_end_effector_frame", "robot1_ee");
  robot2_end_effector_frame_ = nh.param<std::string>("robot2_end_effector_frame", "robot2_ee");

  robot1_tcp_frame_ = nh.param<std::string>("robot1_tcp_frame", "robot1_tcp");
  robot2_tcp_frame_ = nh.param<std::string>("robot2_tcp_frame", "robot2_tcp");

  robot1_grasp_frame_ = nh.param<std::string>("robot1_grasp_frame", "robot1_grasp");
  robot2_grasp_frame_ = nh.param<std::string>("robot2_grasp_frame", "robot2_grasp");

  robot1_planned_pre_grasp_frame_ = nh.param<std::string>("robot1_planned_pre_grasp_frame", "robot1_planned_pre_grasp");
  robot2_planned_pre_grasp_frame_ = nh.param<std::string>("robot2_planned_pre_grasp_frame", "robot2_planned_pre_grasp");

  robot1_planned_grasp_frame_ = nh.param<std::string>("robot1_planned_grasp_frame", "robot1_planned_grasp");
  robot2_planned_grasp_frame_ = nh.param<std::string>("robot2_planned_grasp_frame", "robot2_planned_grasp");

  robot1_planned_lift_frame_ = nh.param<std::string>("robot1_planned_lift_frame", "robot1_planned_lift");
  robot2_planned_lift_frame_ = nh.param<std::string>("robot2_planned_lift_frame", "robot2_planned_lift");

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;
  arms_group_name_ = nh.param<std::string>("arms_group", "arms");
  arms_group_ = std::make_unique<MoveGroupInterface>(arms_group_name_);
  planning_scene_interface_ = std::make_unique<PlanningSceneInterface>();

  std::string robot1_ik_frame_ = arms_group_->getEndEffectorLink();
  std::string robot2_ik_frame_;

  robot1_gripper_command_action_ns_ = nh.param<std::string>("robot1_gripper_command_action_ns", "robot1_gripper_cmd");
  robot1_gripper_command_client_ =
      std::make_unique<GripperCommandActionClient>(robot1_gripper_command_action_ns_, true);
  robot2_gripper_command_action_ns_ = nh.param<std::string>("robot2_gripper_command_action_ns", "robot2_gripper_cmd");
  robot2_gripper_command_client_ =
      std::make_unique<GripperCommandActionClient>(robot2_gripper_command_action_ns_, true);

  object_mesh_topic_ = nh.param<std::string>("object_mesh_topic", "object_mesh");

  planning_attempts_ = nh.param<int>("planning_attempts", 10);
  planning_attempt_time_ = nh.param<double>("planning_attempt_time", 5.);
  pre_grasp_distance_ = nh.param<double>("pre_grasp_distance", 0.1);
  lift_height_ = nh.param<double>("lift_height", 0.2);
  max_velocity_scaling_factor_ = nh.param<double>("max_velocity_scaling_factor", 0.1);
  position_tolerance_ = nh.param<double>("position_tolerance", 0.01);

  touch_links_ = nh.param<std::vector<std::string>>("touch_links", {});
}

void GraspPlanner::prepare()
{
  // Wait until we received the object mesh
  ROS_INFO_STREAM_NAMED("grasp_planner", "Waiting to receive object mesh...");
  object_mesh_ = ros::topic::waitForMessage<shape_msgs::Mesh>(object_mesh_topic_);
  ROS_INFO_STREAM_NAMED("grasp_planner", "Object mesh received.");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  try
  {
    ROS_INFO_STREAM_NAMED("grasp_planner", "Retrieving transform...");

    // Declare all transforms that we need
    tf2::Transform world_to_robot1_grasp, world_to_robot1_pre_grasp, world_to_robot1_lift;
    tf2::Transform world_to_robot2_grasp, world_to_robot2_pre_grasp, world_to_robot2_lift;
    tf2::Transform robot1_grasp_to_pre_grasp, robot2_grasp_to_pre_grasp;
    tf2::Transform robot1_tcp_to_ee, robot2_tcp_to_ee;

    // Retrieve transforms
    geometry_msgs::TransformStamped msg;

    ROS_DEBUG_STREAM_NAMED("grasp_planner", "Looking up transforms.");
    msg = tf_buffer.lookupTransform(world_frame_, robot1_grasp_frame_, ros::Time{ 0 }, ros::Duration{ 120. });
    tf2::fromMsg(msg.transform, world_to_robot1_grasp);

    msg = tf_buffer.lookupTransform(world_frame_, robot2_grasp_frame_, ros::Time{ 0 }, ros::Duration{ 1. });
    tf2::fromMsg(msg.transform, world_to_robot2_grasp);

    msg = tf_buffer.lookupTransform(robot1_tcp_frame_, robot1_end_effector_frame_, ros::Time{ 0 }, ros::Duration{ 1. });
    tf2::fromMsg(msg.transform, robot1_tcp_to_ee);

    msg = tf_buffer.lookupTransform(robot2_tcp_frame_, robot2_end_effector_frame_, ros::Time{ 0 }, ros::Duration{ 1. });
    tf2::fromMsg(msg.transform, robot2_tcp_to_ee);

    // The pre-grasp position is located by translating pre_grasp_distance along
    // the z-axis of the grasp pose
    robot1_grasp_to_pre_grasp.setIdentity();
    robot1_grasp_to_pre_grasp.setOrigin(tf2::Vector3{ 0., 0., -pre_grasp_distance_ });
    world_to_robot1_pre_grasp = world_to_robot1_grasp * robot1_grasp_to_pre_grasp;

    robot2_grasp_to_pre_grasp.setIdentity();
    robot2_grasp_to_pre_grasp.setOrigin(tf2::Vector3{ 0., 0., -pre_grasp_distance_ });
    world_to_robot2_pre_grasp = world_to_robot2_grasp * robot2_grasp_to_pre_grasp;

    // The lift position is just lift_height along the z-axis of the world
    world_to_robot1_lift.setRotation(world_to_robot1_grasp.getRotation());
    world_to_robot1_lift.setOrigin(world_to_robot1_grasp.getOrigin() + tf2::Vector3{ 0., 0., lift_height_ });

    world_to_robot2_lift.setRotation(world_to_robot2_grasp.getRotation());
    world_to_robot2_lift.setOrigin(world_to_robot2_grasp.getOrigin() + tf2::Vector3{ 0., 0., lift_height_ });

    // Express everything in the IK frame because Moveit's target pose is the
    // pose of the ik_frame
    world_to_robot1_pre_grasp_ee_ = world_to_robot1_pre_grasp * robot1_tcp_to_ee;
    world_to_robot2_pre_grasp_ee_ = world_to_robot2_pre_grasp * robot2_tcp_to_ee;

    world_to_robot1_grasp_ee_ = world_to_robot1_grasp * robot1_tcp_to_ee;
    world_to_robot2_grasp_ee_ = world_to_robot2_grasp * robot2_tcp_to_ee;

    world_to_robot1_lift_ee_ = world_to_robot1_lift * robot1_tcp_to_ee;
    world_to_robot2_lift_ee_ = world_to_robot2_lift * robot2_tcp_to_ee;

    // Send planned transforms for visualization
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = world_frame_;

    msg.child_frame_id = robot1_planned_pre_grasp_frame_;
    tf2::convert(world_to_robot1_pre_grasp_ee_, msg.transform);
    broadcaster_.sendTransform(msg);

    msg.child_frame_id = robot2_planned_pre_grasp_frame_;
    tf2::convert(world_to_robot2_pre_grasp_ee_, msg.transform);
    broadcaster_.sendTransform(msg);

    msg.child_frame_id = robot1_planned_grasp_frame_;
    tf2::convert(world_to_robot1_grasp_ee_, msg.transform);
    broadcaster_.sendTransform(msg);

    msg.child_frame_id = robot2_planned_grasp_frame_;
    tf2::convert(world_to_robot2_grasp_ee_, msg.transform);
    broadcaster_.sendTransform(msg);

    msg.child_frame_id = robot1_planned_lift_frame_;
    tf2::convert(world_to_robot1_lift_ee_, msg.transform);
    broadcaster_.sendTransform(msg);

    msg.child_frame_id = robot2_planned_lift_frame_;
    tf2::convert(world_to_robot2_lift_ee_, msg.transform);
    broadcaster_.sendTransform(msg);

    ROS_INFO_STREAM_NAMED("grasp_planner", "Finished retrieving transform.");
  }
  catch (const tf2::TransformException& e)
  {
    throw GraspPlanningException{ "Failed to retrieve grasp pose.\nReason:\n " + std::string{ e.what() } };
  }
}

void GraspPlanner::planPreGrasp()
{
  arms_group_->clearPathConstraints();
  addGroundPlane(*planning_scene_interface_, world_frame_);
  planArmPose(world_to_robot1_pre_grasp_ee_, world_to_robot2_pre_grasp_ee_, "pre-grasp");
}

void GraspPlanner::executePreGrasp()
{
  openGripper();
  auto result = arms_group_->execute(plan_);
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw GraspPlanningException{ "Failed to execute pre-grasp." };
}

void GraspPlanner::planGrasp()
{
  setPathConstraints(world_to_robot1_grasp_ee_, world_to_robot2_grasp_ee_);
  planArmPose(world_to_robot1_grasp_ee_, world_to_robot2_grasp_ee_, "grasp");
}

void GraspPlanner::executeGrasp()
{
  auto result = arms_group_->execute(plan_);
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw GraspPlanningException{ "Failed to execute grasp while moving arm." };

  attachMesh(*planning_scene_interface_, *arms_group_, *object_mesh_, "object_mesh", world_frame_,
             robot1_tcp_frame_, touch_links_);

  closeGripper();
}

void GraspPlanner::planLift()
{
  setPathConstraints(world_to_robot1_lift_ee_, world_to_robot2_lift_ee_);
  planArmPose(world_to_robot1_lift_ee_, world_to_robot2_lift_ee_, "lift");
}

void GraspPlanner::executeLift()
{
  auto result = arms_group_->execute(plan_);
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw GraspPlanningException{ "Failed to execute lift." };
}

void GraspPlanner::stop()
{
  arms_group_->stop();
}

void GraspPlanner::cleanup()
{
  removeGroundPlane(*planning_scene_interface_);
  planning_scene_interface_->removeCollisionObjects({ "object_mesh" });
  arms_group_->clearPathConstraints();
}

void GraspPlanner::planArmPose(const tf2::Transform& robot1_ee_pose, const tf2::Transform& robot2_ee_pose,
                               const std::string& pose_name)
{
  geometry_msgs::Pose robot1_ee_pose_msg, robot2_ee_pose_msg;
  tf2::toMsg(robot1_ee_pose, robot1_ee_pose_msg);
  tf2::toMsg(robot2_ee_pose, robot2_ee_pose_msg);

  arms_group_->setPoseReferenceFrame(world_frame_);
  arms_group_->setPlanningTime(planning_attempt_time_);
  arms_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
  arms_group_->setGoalTolerance(0.01);
  arms_group_->setPoseTarget(robot1_ee_pose_msg, robot1_end_effector_frame_);
  arms_group_->setPoseTarget(robot2_ee_pose_msg, robot2_end_effector_frame_);

  for (int i = 0; i < planning_attempts_; i++)
  {
    auto result = arms_group_->plan(plan_);
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return;
  }

  std::ostringstream msg;
  msg << "Failed to plan " << pose_name << " pose.";
  throw GraspPlanningException{ msg.str() };
}

void GraspPlanner::openGripper()
{
  GripperCommandGoal goal;
  goal.goal = GripperCommandGoal::OPEN;

  robot1_gripper_command_client_->sendGoal(goal);
  robot2_gripper_command_client_->sendGoal(goal);

  robot1_gripper_command_client_->waitForResult(ros::Duration{ 5. });
  robot2_gripper_command_client_->waitForResult(ros::Duration{ 5. });

  if (robot1_gripper_command_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    throw GraspPlanningException{ "Failed to open gripper of robot1." };

  if (robot2_gripper_command_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    throw GraspPlanningException{ "Failed to open gripper of robot2." };
}

void GraspPlanner::closeGripper()
{
  GripperCommandGoal goal;
  goal.goal = GripperCommandGoal::CLOSE;

  robot1_gripper_command_client_->sendGoal(goal);
  robot2_gripper_command_client_->sendGoal(goal);

  robot1_gripper_command_client_->waitForResult(ros::Duration{ 5. });
  robot2_gripper_command_client_->waitForResult(ros::Duration{ 5. });

  if (robot1_gripper_command_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    throw GraspPlanningException{ "Failed to close gripper of robot1." };

  if (robot2_gripper_command_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    throw GraspPlanningException{ "Failed to close gripper of robot2." };
}

void GraspPlanner::setPathConstraints(const tf2::Transform& robot1_goal_pose, const tf2::Transform& robot2_goal_pose)
{
  moveit_msgs::Constraints constraints;
  addPathConstraints(robot1_goal_pose, robot1_end_effector_frame_, world_to_robot1_grasp_ee_, constraints);
  addPathConstraints(robot2_goal_pose, robot2_end_effector_frame_, world_to_robot2_grasp_ee_, constraints);
  arms_group_->setPathConstraints(constraints);
}

void GraspPlanner::addPathConstraints(const tf2::Transform& goal_pose, const std::string& end_effector_frame,
                                      const tf2::Transform& grasp_pose, moveit_msgs::Constraints& constraints)
{
  // Get start pose
  tf2::Transform start_pose;
  auto state = arms_group_->getCurrentState();
  Eigen::Isometry3d arm_to_world = state->getFrameTransform(world_frame_);
  Eigen::Isometry3d arm_to_ee = state->getFrameTransform(end_effector_frame);
  Eigen::Isometry3d world_to_ee = arm_to_world.inverse() * arm_to_ee;
  tf2::convert(world_to_ee, start_pose);
  tf2::Vector3 start_to_goal = goal_pose.getOrigin() - start_pose.getOrigin();

  // Orientation constraint
  const auto& orientation = grasp_pose.getRotation();
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = world_frame_;
  orientation_constraint.link_name = end_effector_frame;
  orientation_constraint.orientation.x = orientation.x();
  orientation_constraint.orientation.y = orientation.y();
  orientation_constraint.orientation.z = orientation.z();
  orientation_constraint.orientation.w = orientation.w();
  orientation_constraint.absolute_x_axis_tolerance = 0.1;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = 0.1;
  orientation_constraint.weight = 1.0;

  // Constrain position to be inside a cylinder
  shape_msgs::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = position_tolerance_;
  cylinder_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] =
      start_to_goal.length() + 2. * position_tolerance_;

  // Cylinder pose
  tf2::Vector3 cylinder_position = start_pose.getOrigin() + 0.5 * start_to_goal;
  tf2::Vector3 cylinder_z_direction = start_to_goal.normalized();
  tf2::Vector3 origin_z_direction{ 0., 0., 1. };
  tf2::Vector3 axis = origin_z_direction.cross(cylinder_z_direction).normalized();
  double angle = std::acos(cylinder_z_direction.dot(origin_z_direction));
  tf2::Quaternion q{ axis, angle };
  q.normalize();
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.position.x = cylinder_position.x();
  cylinder_pose.position.y = cylinder_position.y();
  cylinder_pose.position.z = cylinder_position.z();
  cylinder_pose.orientation.x = q.x();
  cylinder_pose.orientation.y = q.y();
  cylinder_pose.orientation.z = q.z();
  cylinder_pose.orientation.w = q.w();

  // Position constraint
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = world_frame_;
  position_constraint.link_name = end_effector_frame;
  position_constraint.target_point_offset.x = 0.;
  position_constraint.target_point_offset.y = 0.;
  position_constraint.target_point_offset.z = 0.;
  position_constraint.weight = 1.0;
  position_constraint.constraint_region.primitives.push_back(cylinder_primitive);
  position_constraint.constraint_region.primitive_poses.push_back(cylinder_pose);

  // Add to arm group
  constraints.orientation_constraints.push_back(orientation_constraint);
  constraints.position_constraints.push_back(position_constraint);
}

}  // namespace chair_manipulation
