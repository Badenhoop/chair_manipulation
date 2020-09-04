#include "chair_manipulation_grasp_detection_advanced/point_cloud_matcher.h"

namespace chair_manipulation
{
void PointCloudMatcherParameters::load(ros::NodeHandle& nh)
{
}

PointCloudMatcher::PointCloudMatcher(PointCloudMatcherParameters params) : params_(std::move(params))
{
}

void PointCloudMatcher::setInputDatabase(const GraspDatabaseConstPtr& grasp_database)
{
  grasp_database_ = grasp_database;
}

void PointCloudMatcher::setInputCloud(const PointCloudConstPtr& input_cloud)
{
  input_cloud_ = input_cloud;
}

void PointCloudMatcher::match(GraspDatabaseElementConstPtr& element)
{
  // TODO: implement this
  element = grasp_database_->elements_[0];
}

}  // namespace chair_manipulation