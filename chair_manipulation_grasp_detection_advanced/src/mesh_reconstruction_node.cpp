#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction_pipeline.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_reconstruction_node");
  chair_manipulation::runMeshReconstructionPipeline();
  return 0;
}