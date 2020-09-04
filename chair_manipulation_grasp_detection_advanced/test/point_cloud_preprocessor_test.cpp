#include "test_common.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_segmentation.h"

using namespace chair_manipulation;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "point_cloud_preprocessor_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };
  auto point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  auto preprocessed_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_preprocessed", 1);
  auto segmented_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_segmented", 1);
  TestParameters params;

  auto input_cloud = PointCloudPtr{ new PointCloud };
  std::string input_cloud_filename = ros::package::getPath("chair_manipulation_grasp_detection_advanced") + "/test/"
                                                                                                            "clouds/"
                                                                                                            "scene."
                                                                                                            "pcd";
  pcl::io::loadPCDFile(input_cloud_filename, *input_cloud);

  auto preprocessed_point_cloud = PointCloudPtr{ new PointCloud };;
  PointCloudPreprocessor preprocessor{ params.point_cloud_preprocessor_params_ };
  preprocessor.setInputCloud(input_cloud);
  preprocessor.preprocess(*preprocessed_point_cloud);

  auto segmented_cloud = PointCloudPtr{ new PointCloud };
  PointCloudSegmentation segmentation{params.point_cloud_segmentation_params_};
  segmentation.setInputCloud(preprocessed_point_cloud);
  segmentation.segment(*segmented_cloud);

  ROS_INFO_STREAM("Number of points: " << segmented_cloud->size());

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*input_cloud, point_cloud_pub, "world");
    utils::publishPointCloud(*preprocessed_point_cloud, preprocessed_point_cloud_pub, "world");
    utils::publishPointCloud(*segmented_cloud, segmented_point_cloud_pub, "world");
    rate.sleep();
  }

  return 0;
}