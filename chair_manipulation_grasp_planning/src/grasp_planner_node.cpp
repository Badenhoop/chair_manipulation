#include <ros/ros.h>
#include "chair_manipulation_grasp_planning/grasp_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_planner_action_server_node");
  ros::AsyncSpinner spinner{1};
  spinner.start();

  ros::NodeHandle nh_priv{ "~" };
  chair_manipulation::GraspPlanner planner{ nh_priv };

  try {
    ROS_INFO_STREAM_NAMED("grasp_planner", "Preparing");
    planner.prepare();
    ROS_INFO_STREAM_NAMED("grasp_planner", "Planning pre-grasp");
    planner.planPreGrasp();
    ROS_INFO_STREAM_NAMED("grasp_planner", "Executing pre-grasp");
    planner.executePreGrasp();
    ROS_INFO_STREAM_NAMED("grasp_planner", "Planning grasp");
    planner.planGrasp();
    ROS_INFO_STREAM_NAMED("grasp_planner", "Executing grasp");
    planner.executeGrasp();
    ROS_INFO_STREAM_NAMED("grasp_planner", "Planning lift");
    planner.planLift();
    ROS_INFO_STREAM_NAMED("grasp_planner", "Executing lift");
    planner.executeLift();
  }
  catch (const chair_manipulation::GraspPlanningException& e)
  {
    planner.cleanup();
    std::cerr << e.what();
  }
  planner.cleanup();

  ros::waitForShutdown();
  return 0;
}
