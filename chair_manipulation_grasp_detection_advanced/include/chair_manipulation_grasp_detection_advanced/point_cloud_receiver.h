#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_RECEIVER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_RECEIVER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>

namespace chair_manipulation
{
struct PointCloudReceiverParameters
{
  void load(ros::NodeHandle& nh);

  std::vector<std::string> topics_;
  std::string world_frame_;
};

class PointCloudReceiver
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;

  explicit PointCloudReceiver(PointCloudReceiverParameters params);

  void receive(PointCloud& point_cloud);

private:
  PointCloudReceiverParameters params_;
  ros::NodeHandle nh_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_RECEIVER_H
