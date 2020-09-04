#include "test_common.h"
#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h"

using namespace chair_manipulation;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mesh_reconstruction_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{"~"};
  auto mesh_pub = nh.advertise<shape_msgs::Mesh>("mesh", 1);
  auto point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  TestParameters params;

  MeshReconstruction reconstruction{params.mesh_reconstruction_params_ };
  reconstruction.setInputCloud(params.model_->getPointCloud());

  shapes::Mesh shape_mesh;
  reconstruction.reconstruct(shape_mesh);

  shape_msgs::Mesh mesh_msg;
  chair_manipulation::utils::shapeMeshToMsg(shape_mesh, mesh_msg);

  ros::Rate rate{10};
  while (ros::ok())
  {
    mesh_pub.publish(mesh_msg);
    utils::publishPointCloud(*params.model_->getPointCloud(), point_cloud_pub, "world");
    rate.sleep();
  }

  return 0;
}