#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DETECTION_PIPELINE_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DETECTION_PIPELINE_H

#include <ros/ros.h>

namespace chair_manipulation
{
struct GraspDetectionParameters
{
  void load(ros::NodeHandle& nh);

  bool run_once_;
  int num_sample_trials_per_grasp_;
  double sample_radius_;
  std::vector<std::string> grasp_frames_;
};

void runGraspDetectionPipeline();

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DETECTION_PIPELINE_H
