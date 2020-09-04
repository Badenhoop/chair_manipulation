#include "chair_manipulation_grasp_detection_advanced/point_cloud_receiver.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
void PointCloudReceiverParameters::load(ros::NodeHandle& nh)
{
  world_frame_ = nh.param<std::string>("world_frame", "world");

  XmlRpc::XmlRpcValue topic_items;
  if (!nh.getParam("topics", topic_items) || topic_items.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      topic_items.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'topics'." };

  int num_topics = topic_items.size();
  topics_.resize(num_topics);
  for (int i = 0; i < num_topics; i++)
  {
    XmlRpc::XmlRpcValue topic_item = topic_items[i];
    topics_[i] = (std::string)topic_item;
  }
}

PointCloudReceiver::PointCloudReceiver(PointCloudReceiverParameters params)
    : params_(std::move(params)), listener_(buffer_)
{
}

void PointCloudReceiver::receive(PointCloudReceiver::PointCloud& point_cloud)
{
  point_cloud = PointCloud{};
  for (const auto& topic : params_.topics_)
  {
    auto input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_);
    auto input_frame = input_cloud->header.frame_id;
    auto msg = buffer_.lookupTransform(params_.world_frame_, input_frame, ros::Time{ 0 }, ros::Duration{ 1. });
    Eigen::Isometry3d transform = tf2::transformToEigen(msg);

    PointCloud pcl_cloud;
    pcl::fromROSMsg(*input_cloud, pcl_cloud);

    PointCloud transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform.cast<float>());

    point_cloud += transformed_cloud;
  }
}

}  // namespace chair_manipulation