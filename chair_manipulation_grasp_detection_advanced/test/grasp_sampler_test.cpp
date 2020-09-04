#include "test_common.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_sampler_test");
  ros::NodeHandle nh;
  auto point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped msg;
  TestParameters params;
  auto gripper = std::make_shared<Gripper>(params.gripper_params_, params.gripper_urdf_, params.gripper_srdf_);
  auto sampler = std::make_shared<GraspSampler>(params.grasp_sampler_params_, gripper);

  auto point_cloud = params.model_->getPointCloud();
  int index = 12000;
  auto point = (*point_cloud)[index];
  Eigen::Isometry3d grasp_pose;
  bool found = sampler->findGraspPoseAt(point_cloud, point, grasp_pose);

  if (found)
  {
    msg = tf2::eigenToTransform(grasp_pose);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "grasp_pose";
    broadcaster.sendTransform(msg);
  }

  Eigen::Isometry3d point_pose;
  point_pose = Eigen::Translation3d{ point.getVector3fMap().cast<double>() } *
               transform::fromYAxis(point.getNormalVector3fMap().cast<double>());
  msg = tf2::eigenToTransform(point_pose);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.child_frame_id = "point";
  broadcaster.sendTransform(msg);

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*point_cloud, point_cloud_pub, "world");
    rate.sleep();
  }
  return 0;
}