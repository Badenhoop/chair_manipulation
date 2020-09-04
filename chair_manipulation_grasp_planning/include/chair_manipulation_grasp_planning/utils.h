#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_UTILS_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_UTILS_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace chair_manipulation
{
inline double deg2rad(double deg)
{
  return deg * (M_PI / 180.0);
}

void addGroundPlane(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                    const std::string& frame);

void removeGroundPlane(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

void attachMesh(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                moveit::planning_interface::MoveGroupInterface& move_group, const shape_msgs::Mesh& mesh,
                const std::string& id, const std::string& object_frame, const std::string& attach_link_frame,
                const std::vector<std::string>& touch_links);

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_PLANNING_UTILS_H
