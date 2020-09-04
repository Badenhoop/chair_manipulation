#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/create_grasp_database_pipeline.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "create_grasp_database_node");
  chair_manipulation::runCreateGraspDatabasePipeline();
  return 0;
}