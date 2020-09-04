#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/point_cloud_segmentation_pipeline.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "point_cloud_segmentation");
  chair_manipulation::runPointCloudSegmentationPipeline();
  return 0;
}