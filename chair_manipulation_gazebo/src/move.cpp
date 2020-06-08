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

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group)
{
    // BEGIN_SUB_TUTORIAL pick1
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
    // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface &group)
{
    // BEGIN_SUB_TUTORIAL place
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in verbose mode." This is a known issue. |br|
    // |br|
    // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
    // a single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* For place location, we set the value to the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place("object", place_location);
    // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
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
        moveToPose(arm_group, planning_scene_interface, 0.0, -0.5, 0.5, deg2rad(90), 0.0, 0.0);
    else if (command == "close")
        close(gripper_group);
    else if (command == "open")
        open(gripper_group);
    else
        ROS_WARN("Unknown command!");

    ros::shutdown();
    return 0;
}