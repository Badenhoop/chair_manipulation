#include "chair_manipulation_grasp_planning/utils.h"
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
void addGroundPlane(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                    const std::string& frame)
{
  auto object_names = planning_scene_interface.getKnownObjectNames();
  if (std::find(object_names.begin(), object_names.end(), "ground_plane") != object_names.end())
    return;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame;
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = "ground_plane";

  shape_msgs::Plane plane;
  plane.coef[0] = 0.0;
  plane.coef[1] = 0.0;
  plane.coef[2] = 1.0;
  plane.coef[3] = 0.0;

  geometry_msgs::Pose plane_pose;
  plane_pose.orientation.w = 1.0;
  plane_pose.position.x = 0.0;
  plane_pose.position.y = 0.0;
  plane_pose.position.z = -0.01;

  collision_object.planes.push_back(plane);
  collision_object.plane_poses.push_back(plane_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);
}

void removeGroundPlane(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  planning_scene_interface.removeCollisionObjects({ "ground_plane" });
}

void attachMesh(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                moveit::planning_interface::MoveGroupInterface& move_group, const shape_msgs::Mesh& mesh,
                const std::string& id, const std::string& object_frame, const std::string& attach_link_frame,
                const std::vector<std::string>& touch_links)
{
  int operation = moveit_msgs::CollisionObject::ADD;
  auto object_names = planning_scene_interface.getKnownObjectNames();
  if (std::find(object_names.begin(), object_names.end(), id) != object_names.end())
    operation = moveit_msgs::CollisionObject::MOVE;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = object_frame;
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = id;
  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(tf2::toMsg(Eigen::Isometry3d::Identity()));
  collision_object.operation = operation;

  planning_scene_interface.applyCollisionObject(collision_object);
  move_group.attachObject(id, attach_link_frame, touch_links);
}

}  // namespace chair_manipulation