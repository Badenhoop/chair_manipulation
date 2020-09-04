#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/grasp_detection_pipeline.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_detection_node");
  ros::AsyncSpinner spinner{1};
  spinner.start();
  chair_manipulation::runGraspDetectionPipeline();
  ros::waitForShutdown();
  return 0;
}