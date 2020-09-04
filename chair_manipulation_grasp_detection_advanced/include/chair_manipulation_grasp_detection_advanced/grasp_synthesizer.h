#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SYNTHESIZER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SYNTHESIZER_H

#include "grasp_hypothesis.h"
#include "model.h"
#include "multi_arm_grasp.h"
#include "wrench.h"
#include <ros/ros.h>
#include <utility>
#include <Eigen/StdVector>

namespace chair_manipulation
{
struct GraspQuality
{
  GraspQuality()
  {
    epsilon1() = 0.;
    v1() = 0.;
    graspDistance() = 0.;
    nearestArmDistance() = 0.;
    nearestArmOrientation() = 0.;
  }

  std::map<std::string, double> values_;

  double& epsilon1()
  {
    return values_["epsilon1"];
  }

  double epsilon1() const
  {
    return values_.at("epsilon1");
  }

  double& v1()
  {
    return values_["v1"];
  }

  double v1() const
  {
    return values_.at("v1");
  }

  double& graspDistance()
  {
    return values_["grasp_distance"];
  }

  double graspDistance() const
  {
    return values_.at("grasp_distance");
  }

  double& nearestArmDistance()
  {
    return values_["nearest_arm_distance"];
  }

  double nearestArmDistance() const
  {
    return values_.at("nearest_arm_distance");
  }

  double& nearestArmOrientation()
  {
    return values_["nearest_arm_orientation"];
  }

  double nearestArmOrientation() const
  {
    return values_.at("nearest_arm_orientation");
  }

  GraspQuality operator*(GraspQuality& other) const
  {
    GraspQuality product;
    for (const auto& pair : values_)
    {
      const auto& key = pair.first;
      product.values_[key] = values_.at(key) * other.values_.at(key);
    }
    return product;
  }

  double sum() const
  {
    double result = 0.;
    for (const auto& pair : values_)
      result += pair.second;
    return result;
  }
};

struct GraspQualityWeights : GraspQuality
{
  void load(ros::NodeHandle& nh);
};

struct GraspSynthesizerParameters
{
  void load(ros::NodeHandle& nh);

  int num_arms_;
  double friction_coefficient_;
  int num_friction_edges_;
  double max_arm_radius_;
  double max_yaw_angle_;
  double min_height_;
  double nms_distance_threshold_;
  double nms_orientation_threshold_;
  std::string world_frame_;
  std::vector<std::string> arm_base_frames_;
};

class GraspSynthesizer
{
public:
  using GraspCandidate = std::vector<const GraspHypothesis*>;

  GraspSynthesizer(GraspSynthesizerParameters params, GraspQualityWeights weights);

  void generateGraspCandidates(const std::vector<GraspHypothesis>& hypotheses, std::vector<GraspCandidate>& candidates);

  void synthesize(const std::vector<GraspCandidate>& candidates, const Model& model, std::size_t max_num_grasps,
                  std::vector<MultiArmGrasp>& synthesized_grasps) const;

private:
  GraspSynthesizerParameters params_;
  GraspQualityWeights weights_;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> arm_base_poses_;

  void generateGraspCandidatesRecursively(const std::vector<GraspHypothesis>& hypotheses,
                                          std::vector<GraspCandidate>& candidates,
                                          const GraspCandidate& curr_candidate = GraspCandidate{},
                                          std::size_t arm_index = 0, std::size_t hypothesis_index = 0) const;

  bool checkMinHeight(const GraspCandidate& candidate) const;

  bool computeWrenchSpaceQualities(const GraspCandidate& candidate, const Model& model, GraspQuality& quality) const;

  bool computeGraspDistanceQuality(const GraspCandidate& candidate, const Model& model, GraspQuality& quality) const;

  bool computeNearestArmQualities(const GraspCandidate& candidate, GraspQuality& quality) const;

  void nonMaximumSuppression(std::vector<MultiArmGrasp>& synthesized_grasps) const;
};

using GraspSynthesizerPtr = std::shared_ptr<GraspSynthesizer>;
using GraspSynthesizerConstPtr = std::shared_ptr<const GraspSynthesizer>;

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SYNTHESIZER_H
