#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <pcl/visualization/pcl_visualizer.h>

using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointNormalCloudPtr = PointNormalCloud::Ptr;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pcl_viewer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };

  auto cloud_topic = nh_priv.param<std::string>("cloud_topic", "cloud");
  auto cloud2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic);
  auto cloud = PointNormalCloudPtr{ new PointNormalCloud };
  pcl::fromROSMsg(*cloud2, *cloud);

  pcl::visualization::PCLVisualizer vis("PCL viewer");
  vis.addPointCloud<pcl::PointNormal>(cloud);
  vis.addPointCloudNormals<pcl::PointNormal>(cloud, 1, 0.02f, "cloud_normals");
  vis.spin();

  return 0;
}