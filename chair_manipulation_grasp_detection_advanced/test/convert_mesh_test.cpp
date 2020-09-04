#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <pcl/io/vtk_lib_io.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "convert_mesh_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };
  auto mesh_pub = nh.advertise<shape_msgs::Mesh>("mesh", 1);

  pcl::PolygonMesh polygon_mesh;
  auto mesh_filename =
      nh_priv.param<std::string>("mesh", ros::package::getPath("chair_manipulation_chair_models") + "/models/"
                                                                                                    "dining_chair/"
                                                                                                    "meshes/"
                                                                                                    "dining_chair.ply");
  if (!pcl::io::loadPolygonFilePLY(mesh_filename, polygon_mesh))
  {
    ROS_ERROR("Failed to load mesh.");
    return -1;
  }

  shapes::Mesh shape_mesh;
  ROS_INFO("Converting pcl::PolygonMesh to shapes::Mesh");
  chair_manipulation::utils::polygonToShapeMesh(polygon_mesh, shape_mesh);

  shape_msgs::Mesh mesh_msg;
  ROS_INFO("Converting shapes::Mesh to shape_msgs::Mesh");
  chair_manipulation::utils::shapeMeshToMsg(shape_mesh, mesh_msg);

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    mesh_pub.publish(mesh_msg);
    rate.sleep();
  }

  return 0;
}