#include <chair_manipulation_grasp_detection_advanced/utils.h>
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"

namespace chair_manipulation
{
void GraspDatabaseCreatorParameters::load(ros::NodeHandle& nh)
{
  num_sample_trials_per_model_ = nh.param<int>("num_sample_trials_per_model", 1000);
  min_num_grasps_per_model_ = nh.param<int>("min_num_grasps_per_model", 3);
  max_num_grasps_per_model_ = nh.param<int>("max_num_grasps_per_model", 10);

  XmlRpc::XmlRpcValue models_array;
  if (!nh.getParam("models", models_array) || models_array.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      models_array.size() == 0)
  {
    throw exception::Parameter{ "No models specified." };
  }
  int num_models = models_array.size();
  mesh_filenames_.resize(num_models);
  point_cloud_filenames_.resize(num_models);
  for (int i = 0; i < num_models; i++)
  {
    auto model_item = models_array[i];
    auto mesh_filename = utils::loadStringParameter(model_item, "mesh");
    auto point_cloud_filename = utils::loadStringParameter(model_item, "point_cloud");
    mesh_filenames_[i] = mesh_filename;
    point_cloud_filenames_[i] = point_cloud_filename;
  }
}

void GraspDatabaseCreator::createGraspDatabase(GraspDatabase& database)
{
  for (std::size_t i = 0; i < params_.mesh_filenames_.size(); i++)
  {
    const auto& mesh_filename = params_.mesh_filenames_[i];
    const auto& point_cloud_filename = params_.point_cloud_filenames_[i];
    auto model = std::make_shared<Model>(mesh_filename, point_cloud_filename);
    auto element = std::make_shared<GraspDatabaseElement>();
    std::vector<GraspHypothesis> grasp_hypotheses;
    grasp_sampler_->sampleGraspHypotheses(*model, params_.num_sample_trials_per_model_, grasp_hypotheses);
    std::vector<GraspCandidate> candidates;
    grasp_synthesizer_->generateGraspCandidates(grasp_hypotheses, candidates);
    grasp_synthesizer_->synthesize(candidates, *model, params_.max_num_grasps_per_model_, element->grasps_);
    if (element->grasps_.size() < params_.min_num_grasps_per_model_)
    {
      ROS_WARN_STREAM_NAMED("grasp_database_creator",
                            "Unable to find the minimum number of grasps for the model with mesh '"
                                << mesh_filename << "' and point cloud '" << point_cloud_filename << "'.");
      continue;
    }
    element->mesh_filename_ = mesh_filename;
    element->point_cloud_filename_ = point_cloud_filename;
    element->model_ = model;
    database.elements_.push_back(element);
  }
}

}  // namespace chair_manipulation