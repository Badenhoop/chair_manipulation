#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SAMPLER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SAMPLER_H

#include "model.h"
#include "grasp_hypothesis.h"
#include "multi_arm_grasp.h"
#include "gripper.h"
#include "ros/ros.h"
#include <random>
#include <pcl/search/kdtree.h>
#include <geometry_msgs/Pose.h>

namespace chair_manipulation
{
struct GraspSamplerParameters
{
  void load(ros::NodeHandle& nh);

  double max_antipodal_normal_angle_;
  double max_antipodal_position_angle_;
  double max_palm_normal_angle_;
  double gripper_pad_distance_;
  double gripper_pad_length_;
};

class GraspSampler
{
public:
  using PointCloud = Model::PointCloud;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;
  using PointT = PointCloud::PointType;

  GraspSampler(GraspSamplerParameters params, GripperPtr gripper)
    : params_(std::move(params)), gripper_(std::move(gripper))
  {
  }

  void sampleGraspHypotheses(const Model& model, std::size_t sample_trials, std::vector<GraspHypothesis>& hypotheses);

  void sampleGraspHypothesesFromPrior(const Model& model, const std::vector<MultiArmGrasp>& prior_grasps,
                                      std::size_t sample_trials_per_grasp, double sample_radius,
                                      std::vector<GraspHypothesis>& hypotheses);

  void filterCollisionFree(const Model& model, const std::vector<MultiArmGrasp>& grasps,
                           std::vector<GraspHypothesis>& hypotheses);

  bool sampleGraspPose(const PointCloudConstPtr& point_cloud, Eigen::Isometry3d& grasp_pose);

  bool findGraspPoseAt(const PointCloudConstPtr& point_cloud, const PointT& point, Eigen::Isometry3d& grasp_pose);

private:
  struct CollisionCheckingScope
  {
    CollisionCheckingScope(GraspSampler& sampler, const Model& model) : sampler_(sampler)
    {
      sampler_.prepareCollisionChecking(model);
    }

    ~CollisionCheckingScope()
    {
      sampler_.cleanupCollisionChecking();
    }

    GraspSampler& sampler_;
  };

  GraspSamplerParameters params_;
  GripperPtr gripper_;

  std::default_random_engine random_generator_;

  using SearchMethod = pcl::search::KdTree<PointCloud::PointType>;
  SearchMethod search_method_;

  void prepareCollisionChecking(const Model& model);

  void cleanupCollisionChecking();

  bool computeContacts(const Eigen::Isometry3d& pose, std::vector<Contact>& contacts);

  /**
   * Angle between the normal of the reference point and the normal of the antipodal point
   */
  double computeAntipodalNormalAngle(const PointT& reference_point, const PointT& antipodal_point) const;

  /**
   * Angle between the normal of the reference point and the connecting line from the antipodal point
   * to the reference point
   */
  double computeAntipodalPositionAngle(const PointT& reference_point, const PointT& antipodal_point) const;

  /**
   * Angle between the normal of reference point and the normal of the palm point minus pi/2
   */
  double computePalmNormalAngle(const PointT& reference_point, const PointT& palm_point) const;

  /**
   * The cost associated with some antipodal point
   */
  double computeAntipodalCost(const PointT& reference_point, const PointT& antipodal_point) const;

  /**
   * The cost associated with some palm point
   */
  double computePalmCost(const PointT& reference_point, const PointT& palm_point) const;
};

using GraspSamplerPtr = std::shared_ptr<GraspSampler>;
using GraspSamplerConstPtr = std::shared_ptr<const GraspSampler>;

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SAMPLER_H
