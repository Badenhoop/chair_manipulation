#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_CREATOR_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_CREATOR_H

#include "model.h"
#include "grasp_database.h"
#include "grasp_sampler.h"
#include "grasp_synthesizer.h"
#include "ros/ros.h"
#include <random>
#include <utility>
#include <pcl/search/kdtree.h>

namespace chair_manipulation
{
struct GraspDatabaseCreatorParameters
{
  void load(ros::NodeHandle& nh);

  int num_sample_trials_per_model_;
  int min_num_grasps_per_model_;
  int max_num_grasps_per_model_;
  std::vector<std::string> mesh_filenames_;
  std::vector<std::string> point_cloud_filenames_;
};

class GraspDatabaseCreator
{
public:
  using GraspCandidate = GraspSynthesizer::GraspCandidate;

  GraspDatabaseCreator(GraspDatabaseCreatorParameters params, GraspSamplerPtr grasp_sampler,
                       GraspSynthesizerPtr grasp_synthesizer)
    : params_(std::move(params))
    , grasp_sampler_(std::move(grasp_sampler))
    , grasp_synthesizer_(std::move(grasp_synthesizer))
  {
  }

  void createGraspDatabase(GraspDatabase& database);

private:
  GraspDatabaseCreatorParameters params_;
  GraspSamplerPtr grasp_sampler_;
  GraspSynthesizerPtr grasp_synthesizer_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_CREATOR_H
