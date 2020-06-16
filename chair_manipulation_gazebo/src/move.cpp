#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>

double deg2rad(double deg)
{
    return deg * (M_PI / 180.0);
}

void addGroundPlane(moveit::planning_interface::MoveGroupInterface &group,
                    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();
    collision_object.id = "ground_plane";

    shape_msgs::Plane plane;
    plane.coef[0] = 0.0;
    plane.coef[1] = 0.0;
    plane.coef[2] = 1.0;
    plane.coef[3] = 0.0;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.1;

    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);
}

void moveToPose(moveit::planning_interface::MoveGroupInterface &group,
                moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                double x, double y, double z, double roll, double pitch, double yaw)
{
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::Pose target_pose;
    target_pose.orientation = tf2::toMsg(orientation);;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    group.setPoseTarget(target_pose);

    addGroundPlane(group, planning_scene_interface);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move", "Visualizing plan %s", success ? "" : "FAILED");
    group.move();
}

void moveUp(moveit::planning_interface::MoveGroupInterface &group)
{
    group.setNamedTarget("up");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move", "Visualizing plan %s", success ? "" : "FAILED");
    group.move();
}

void open(moveit::planning_interface::MoveGroupInterface &group)
{
    group.setNamedTarget("open");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    group.plan(plan);
    group.move();
}

void close(moveit::planning_interface::MoveGroupInterface &group)
{
    group.setNamedTarget("closed");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    group.plan(plan);
    group.move();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string ARM_GROUP = "arm";
    static const std::string GRIPPER_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group(GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm_group.setPlanningTime(60.0);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "move", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    std::string command;
    if (!ros::param::get("~command", command))
    {
        ROS_ERROR("Parameter 'command' not specified!");
        return -1;
    }

    if (command == "move_up")
        moveUp(arm_group);
    else if (command == "move_to_pose")
        moveToPose(arm_group, planning_scene_interface, 0.0, -0.5, 0.5, deg2rad(180), deg2rad(0), deg2rad(0));
    else if (command == "close")
        close(gripper_group);
    else if (command == "open")
        open(gripper_group);
    else
        ROS_WARN("Unknown command!");

    ros::shutdown();
    return 0;
}