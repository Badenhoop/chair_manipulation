#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_MATCHER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_MATCHER_H

#include <ros/ros.h>
#include "grasp_database.h"
#include <pcl/features/impl/fpfh.hpp>

namespace chair_manipulation
{
struct PointCloudMatcherParameters
{
  void load(ros::NodeHandle& nh);
};

class PointCloudMatcher
{
public:
  using PointT = pcl::PointNormal;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  explicit PointCloudMatcher(PointCloudMatcherParameters params);

  void setInputDatabase(const GraspDatabaseConstPtr& grasp_database);

  void setInputCloud(const PointCloudConstPtr& input_cloud);

  void match(GraspDatabaseElementConstPtr& element);

private:
  PointCloudMatcherParameters params_;
  GraspDatabaseConstPtr grasp_database_;
  PointCloudConstPtr input_cloud_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_MATCHER_H
