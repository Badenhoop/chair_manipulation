#include "chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/statistics.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
void GraspQualityWeights::load(ros::NodeHandle& nh)
{
  epsilon1() = nh.param<double>("epsilon1", 1.0);
  v1() = nh.param<double>("v1", 1.0);
  graspDistance() = nh.param<double>("grasp_distance", 1.0);
  nearestArmDistance() = nh.param<double>("nearest_arm_distance", 1.0);
  nearestArmOrientation() = nh.param<double>("nearest_arm_orientation", 1.0);
}

void GraspSynthesizerParameters::load(ros::NodeHandle& nh)
{
  num_arms_ = nh.param<int>("num_arms", 2);
  friction_coefficient_ = nh.param<double>("friction_coefficient", 0.8);
  num_friction_edges_ = nh.param<int>("num_friction_edges", 8);
  max_arm_radius_ = nh.param<double>("max_arm_radius", 1.0);
  max_yaw_angle_ = nh.param<double>("max_yaw_angle", M_PI_2);
  min_height_ = nh.param<double>("min_height", 0.);
  nms_distance_threshold_ = nh.param<double>("nms_distance_threshold", 0.1);
  nms_orientation_threshold_ = nh.param<double>("nms_orientation_threshold", 0.1);
  world_frame_ = nh.param<std::string>("world_frame", "world");

  XmlRpc::XmlRpcValue arm_base_frames_array;
  if (!nh.getParam("arm_base_frames", arm_base_frames_array) ||
      arm_base_frames_array.getType() != XmlRpc::XmlRpcValue::TypeArray || arm_base_frames_array.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'arm_base_frames'." };

  int num_base_frames = arm_base_frames_array.size();
  arm_base_frames_.resize(num_base_frames);
  for (int i = 0; i < num_base_frames; i++)
  {
    XmlRpc::XmlRpcValue item = arm_base_frames_array[i];
    arm_base_frames_[i] = (std::string)item;
  }
}

GraspSynthesizer::GraspSynthesizer(GraspSynthesizerParameters params, GraspQualityWeights weights)
  : params_(std::move(params)), weights_(std::move(weights))
{
  if (weights_.nearestArmDistance() != 0. || weights_.nearestArmOrientation() != 0.)
  {
    // Get arm base poses (with respect to the world frame) using tf2
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    arm_base_poses_.resize(params.num_arms_);
    for (std::size_t i = 0; i < params_.num_arms_; i++)
    {
      const auto& arm_base_frame = params_.arm_base_frames_[i];
      auto msg = tf_buffer.lookupTransform(params_.world_frame_, arm_base_frame, ros::Time{ 0 }, ros::Duration{ 3. });
      arm_base_poses_[i] = tf2::transformToEigen(msg);
    }
  }
}

void GraspSynthesizer::generateGraspCandidates(const std::vector<GraspHypothesis>& hypotheses,
                                               std::vector<GraspCandidate>& candidates)
{
  if (hypotheses.size() < params_.num_arms_)
    throw exception::IllegalArgument{ "The number of hypotheses must be at least as large as the number of arms." };

  generateGraspCandidatesRecursively(hypotheses, candidates);
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Generated " << candidates.size() << " grasp candidates.");
}

void GraspSynthesizer::synthesize(const std::vector<GraspCandidate>& candidates, const Model& model,
                                  std::size_t max_num_grasps, std::vector<MultiArmGrasp>& synthesized_grasps) const
{
  Stopwatch stopwatch;

  double weight_sum = weights_.sum();
  if (weight_sum == 0)
    throw exception::Parameter{ "The sum of the weights must not be zero." };

  std::map<std::string, std::vector<double>> values, weighted_values;
  std::vector<double> quality_scores;

  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Start scoring grasp candidates.");
  for (std::size_t i = 0; i < candidates.size(); i++)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "");
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "=== Candidate " << (i + 1) << "/" << candidates.size() << " ===");
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "");

    stopwatch.start();

    const auto& candidate = candidates[i];
    GraspQuality quality;

    for (std::size_t j = 0; j < candidate.size(); j++)
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer",
                             "pose of arm " << j << ": [" << utils::poseToStr(candidate[j]->pose_) << "]");

    if (!checkMinHeight(candidate) || !computeWrenchSpaceQualities(candidate, model, quality) ||
        !computeGraspDistanceQuality(candidate, model, quality) || !computeNearestArmQualities(candidate, quality))
      continue;

    auto weighted_quality = weights_ * quality;
    double quality_score = weighted_quality.sum() / weight_sum;

    MultiArmGrasp grasp;
    grasp.quality_ = quality_score;
    for (const auto& hypothesis : candidate)
      grasp.poses_.push_back(hypothesis->pose_);

    synthesized_grasps.push_back(grasp);

    // Store values for computing statistics at the end
    for (const auto& pair : weights_.values_)
    {
      const auto& key = pair.first;
      double value = quality.values_[key];
      double weighted_value = weighted_quality.values_[key];
      values[key].push_back(value);
      weighted_values[key].push_back(weighted_value);
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", key << ": " << value);
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", key << " weighted: " << weighted_value);
    }
    quality_scores.push_back(quality_score);
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "grasp quality: " << quality_score);

    stopwatch.stop();
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer",
                           "Processing current candidate took " << stopwatch.elapsedSeconds() << "s.");
  }

  if (synthesized_grasps.empty())
  {
    ROS_WARN_STREAM_NAMED("grasp_synthesizer", "Failed to synthesize any valid grasp candidates.");
    return;
  }

  std::size_t num_grasps_before = synthesized_grasps.size();
  nonMaximumSuppression(synthesized_grasps);
  std::size_t num_grasps_after = synthesized_grasps.size();
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Filtered out " << (num_grasps_before - num_grasps_after) << " grasps.");

  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Sorting remaining grasps by grasp quality in "
                                              "descending order.");
  std::sort(synthesized_grasps.begin(), synthesized_grasps.end(),
            [&](const auto& grasp1, const auto& grasp2) { return grasp1.quality_ > grasp2.quality_; });

  if (synthesized_grasps.size() > max_num_grasps)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer",
                           "Need to clip the number of returned grasps to " << max_num_grasps << ".");
    synthesized_grasps.resize(max_num_grasps);
  }

  for (auto& pair : values)
    statistics::debugSummary(pair.second, pair.first);

  for (auto& pair : weighted_values)
    statistics::debugSummary(pair.second, pair.first + " weighted");

  statistics::debugSummary(quality_scores, "grasp_quality");
}

void GraspSynthesizer::generateGraspCandidatesRecursively(const std::vector<GraspHypothesis>& hypotheses,
                                                          std::vector<GraspCandidate>& candidates,
                                                          const GraspCandidate& curr_candidate, std::size_t arm_index,
                                                          std::size_t hypothesis_index) const
{
  for (std::size_t j = hypothesis_index; j < hypotheses.size() - params_.num_arms_ + arm_index + 1; j++)
  {
    auto new_candidate = curr_candidate;
    auto hypothesis = &hypotheses[j];
    new_candidate.push_back(hypothesis);
    if (arm_index == params_.num_arms_ - 1)
      candidates.push_back(new_candidate);
    else
      generateGraspCandidatesRecursively(hypotheses, candidates, new_candidate, arm_index + 1, j + 1);
  }
}

bool GraspSynthesizer::checkMinHeight(const GraspCandidate& candidate) const
{
  for (const auto& grasp : candidate)
  {
    // Filter out if grasp position is too low
    if (grasp->pose_.translation().z() < params_.min_height_)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Grasp position is too low - omit candidate.");
      return false;
    }
  }
  return true;
}

bool GraspSynthesizer::computeWrenchSpaceQualities(const GraspCandidate& candidate, const Model& model,
                                                   GraspQuality& quality) const
{
  if (weights_.epsilon1() == 0. && weights_.v1() == 0.)
    return true;

  Stopwatch stopwatch;
  stopwatch.start();

  // Extract all contacts from the grasp hypotheses of this candidate
  std::size_t num_contacts = 0;
  for (const auto& hypothesis : candidate)
    num_contacts += hypothesis->contacts_.size();

  std::vector<Contact> contacts;
  contacts.resize(num_contacts);

  std::size_t i = 0;
  for (const auto& hypothesis : candidate)
  {
    for (const auto& contact : hypothesis->contacts_)
      contacts[i++] = contact;
  }

  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "contacts: [" << utils::contactsToStr(contacts) << "]");

  WrenchSpace wrench_space(contacts, model, params_.friction_coefficient_, params_.num_friction_edges_);

  stopwatch.stop();
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Computed wrench space.");
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "It took " << stopwatch.elapsedSeconds() << "s.");

  if (!wrench_space.isForceClosure())
  {
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "No force closure - omit candidate.");
    return false;
  }

  quality.epsilon1() = wrench_space.getEpsilon1Quality();
  quality.v1() = wrench_space.getV1Quality();
  return true;
}

bool GraspSynthesizer::computeGraspDistanceQuality(const GraspCandidate& candidate, const Model& model,
                                                   GraspQuality& quality) const
{
  if (weights_.graspDistance() == 0.)
    return true;

  double max_distance = (model.getMax() - model.getMin()).norm();
  double sum = 0.;
  std::size_t n = 0;
  for (std::size_t i = 0; i < candidate.size(); i++)
  {
    for (std::size_t j = 0; j < candidate.size(); j++)
    {
      sum += (candidate[i]->pose_.translation() - candidate[j]->pose_.translation()).norm() / max_distance;
      n++;
    }
  }
  quality.graspDistance() = sum / n;
  return true;
}

bool GraspSynthesizer::computeNearestArmQualities(const GraspCandidate& candidate, GraspQuality& quality) const
{
  if (weights_.nearestArmDistance() == 0. && weights_.nearestArmOrientation() == 0.)
    return true;

  double nearest_arm_distance = 0.;
  double nearest_arm_orientation = 0.;

  // We want every grasp to be assigned to a unique arm so this tells us whether grasp i
  // is already assigned to arm i where we assign grasp i to the arm that is closest to it.
  std::vector<bool> assigned(params_.num_arms_, false);

  for (const auto& grasp : candidate)
  {
    const auto& grasp_pose = grasp->pose_;

    // Get nearest arm.
    // We only consider the distance on the xy-plane.
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector3d min_arm_to_grasp_xy;
    std::size_t arm_index = 0;
    for (std::size_t i = 0; i < params_.num_arms_; i++)
    {
      const auto& arm_base_pose = arm_base_poses_[i];
      Eigen::Vector3d arm_to_grasp = (grasp_pose.translation() - arm_base_pose.translation());
      Eigen::Vector3d arm_to_grasp_xy = arm_to_grasp - utils::projection(Eigen::Vector3d::UnitZ().eval(), arm_to_grasp);
      double distance = arm_to_grasp_xy.norm();
      if (distance < min_distance)
      {
        min_distance = distance;
        min_arm_to_grasp_xy = arm_to_grasp_xy;
        arm_index = i;
      }
    }

    // If the arm is already assigned we reject the grasp by setting output to negative infinity
    if (assigned[arm_index])
    {
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Nearest arm assigned more than once - omit candidate.");
      return false;
    }

    // We also reject this candidate if the distance to the nearest arm exceeds the given maximum
    if (min_distance > params_.max_arm_radius_)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Exceeding maximum distance to nearest robot - omit candidate.");
      return false;
    }

    // We compute the angle between the grasps' z-direction and the vector that
    // goes from the base to the grasp where both vectors are projected onto the xy-plane.
    Eigen::Vector3d grasp_direction = grasp_pose.rotation().col(2);
    Eigen::Vector3d grasp_direction_xy =
        grasp_direction - utils::projection(Eigen::Vector3d::UnitZ().eval(), grasp_direction);
    double yaw_angle = std::acos(min_arm_to_grasp_xy.normalized().dot(grasp_direction_xy.normalized()));
    if (yaw_angle > params_.max_yaw_angle_)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Exceeding maximum yaw angle - omit candidate.");
      return false;
    }

    assigned[arm_index] = true;

    nearest_arm_distance += 1. - (min_distance / params_.max_arm_radius_);
    nearest_arm_orientation += 1. - (yaw_angle / params_.max_yaw_angle_);
  }
  quality.nearestArmDistance() = nearest_arm_distance / params_.num_arms_;
  quality.nearestArmOrientation() = nearest_arm_orientation / params_.num_arms_;
  return true;
}

void GraspSynthesizer::nonMaximumSuppression(std::vector<MultiArmGrasp>& synthesized_grasps) const
{
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Performing non maximum suppression.");
  std::vector<MultiArmGrasp> filtered_grasps;
  for (std::size_t i = 0; i < synthesized_grasps.size(); i++)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Processing " << (i + 1) << "/" << synthesized_grasps.size());
    const auto& grasp_i = synthesized_grasps[i];
    bool is_max = true;
    for (std::size_t j = 0; j < synthesized_grasps.size(); j++)
    {
      if (i == j)
        continue;

      const auto& grasp_j = synthesized_grasps[j];

      // We are only interested in pairs that look similar
      bool all_similar = true;
      for (std::size_t k = 0; k < params_.num_arms_; k++)
      {
        const auto& pose_i = grasp_i.poses_[k];
        Eigen::Vector3d z_axis_i = pose_i.rotation().col(2);
        // A similar pose could be at any arm index in the array
        bool exists_similar = false;
        for (std::size_t l = 0; l < params_.num_arms_; l++)
        {
          const auto& pose_j = grasp_j.poses_[l];
          double distance = (pose_i.translation() - pose_j.translation()).norm();
          // Consider the angle between the z-axes
          Eigen::Vector3d z_axis_j = pose_j.rotation().col(2);
          double angle = std::acos(z_axis_i.dot(z_axis_j));
          if (distance < params_.nms_distance_threshold_ && angle < params_.nms_orientation_threshold_)
          {
            exists_similar = true;
            break;
          }
        }

        if (!exists_similar)
        {
          all_similar = false;
          break;
        }
      }

      if (all_similar && grasp_j.quality_ > grasp_i.quality_)
      {
        is_max = false;
        break;
      }
    }

    if (is_max)
      filtered_grasps.push_back(grasp_i);
  }
  synthesized_grasps = filtered_grasps;
}

}  // namespace chair_manipulation