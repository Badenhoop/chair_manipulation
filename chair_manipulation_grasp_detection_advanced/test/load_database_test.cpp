#include "test_common.h"
#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "load_database_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner{ 1 };
  tf2_ros::StaticTransformBroadcaster broadcaster;
  auto point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  auto mesh_pub = nh.advertise<shape_msgs::Mesh>("mesh", 1);
  TestParameters params;

  GraspDatabase database;
  ros::NodeHandle database_nh{ "grasp_database" };
  database.load(database_nh);

  const auto& element = database.elements_[0];
  for (std::size_t i = 0; i < element->grasps_.size(); i++)
  {
    const auto& grasp = element->grasps_[i];
    for (std::size_t j = 0; j < grasp.poses_.size(); j++)
    {
      const auto& pose = grasp.poses_[j];
      auto tf_msg = tf2::eigenToTransform(pose);
      tf_msg.header.stamp = ros::Time::now();
      tf_msg.header.frame_id = "world";
      tf_msg.child_frame_id = "grasp(" + std::to_string(i) + "," + std::to_string(j) + ")";
      broadcaster.sendTransform(tf_msg);
    }
  }

  shape_msgs::Mesh mesh_msg;
  chair_manipulation::utils::shapeMeshToMsg(*element->model_->getMesh(), mesh_msg);

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*element->model_->getPointCloud(), point_cloud_pub, "world");
    mesh_pub.publish(mesh_msg);
    rate.sleep();
  }

  return 0;
}