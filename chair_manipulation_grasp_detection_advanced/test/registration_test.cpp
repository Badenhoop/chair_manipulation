#include "test_common.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace chair_manipulation;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointNormalCloudPtr = PointNormalCloud::Ptr;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "registration_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };
  auto source_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_source", 1);
  auto target_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_target", 1);
  auto aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_registered", 1);
  TestParameters params;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  auto source_normal_cloud = params.model_->getPointCloud();

  auto target_cloud = PointCloudPtr{ new PointCloud };
  std::string cloud_filename = ros::package::getPath("chair_manipulation_grasp_detection_advanced") + "/test/clouds/"
                                                                                                      "chair_segmented."
                                                                                                      "pcd";

  pcl::io::loadPCDFile(cloud_filename, *target_cloud);

  auto filtered_target_cloud = PointCloudPtr{ new PointCloud };
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(0.03, 0.03, 0.03);
  voxel_filter.setInputCloud(target_cloud);
  voxel_filter.filter(*filtered_target_cloud);

  auto target_normal_cloud = PointNormalCloudPtr{ new PointNormalCloud };
  PointCloudPreprocessor preprocessor{ params.point_cloud_preprocessor_params_ };
  preprocessor.setInputCloud(filtered_target_cloud);
  preprocessor.estimateNormals(*target_normal_cloud);

  NonrigidTransform transform;
  auto aligned_cloud = PointNormalCloudPtr{ new PointNormalCloud };
  PointCloudRegistration registration{ params.point_cloud_registration_params_ };
  registration.setInputSource(source_normal_cloud);
  registration.setInputTarget(target_normal_cloud);
  registration.align(*aligned_cloud, transform);

  Eigen::Isometry3d original_pose = utils::poseFromStr("0.3743 0.3447 0.43 0 0 0 1");
  Eigen::Isometry3d transformed_pose = transform * original_pose;

  visual_tools.publishArrow(original_pose, rvt::BLUE);
  visual_tools.publishArrow(transformed_pose, rvt::RED);

  visual_tools.trigger();

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*source_normal_cloud, source_cloud_pub, "world");
    utils::publishPointCloud(*target_normal_cloud, target_cloud_pub, "world");
    utils::publishPointCloud(*aligned_cloud, aligned_cloud_pub, "world");
    rate.sleep();
  }

  return 0;
}