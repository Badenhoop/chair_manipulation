#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H

#include "chair_manipulation_grasp_detection_advanced/contact.h"
#include "chair_manipulation_grasp_detection_advanced/multi_arm_grasp.h"
#include "chair_manipulation_grasp_detection_advanced/nonrigid_transform.h"
#include <pcl/PolygonMesh.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <vtkPolyDataMapper.h>

namespace chair_manipulation
{
namespace utils
{
void polygonToShapeMesh(const pcl::PolygonMesh& polygon_mesh, shapes::Mesh& shape_mesh);

void polydataToShapeMesh(vtkPolyData* polydata, shapes::Mesh& shape_mesh);

void shapeMeshToMsg(const shapes::Mesh& shape_mesh, shape_msgs::Mesh& msg);

void polygonMeshToMsg(const pcl::PolygonMesh& polygon_mesh, shape_msgs::Mesh& msg);

std::string loadStringParameter(const XmlRpc::XmlRpcValue& value, const std::string& key);

double loadDoubleParameter(const XmlRpc::XmlRpcValue& value, const std::string& key);

std::string poseToStr(const Eigen::Isometry3d& pose);

Eigen::Isometry3d poseFromStr(const std::string& str);

std::string vectorToStr(const Eigen::Vector3d& vector);

Eigen::Vector3d vectorFromStr(const std::string& str);

std::string contactsToStr(const std::vector<Contact>& contacts);

std::vector<Contact> contactsFromStr(const std::string& str);

template <typename T>
bool contains(const std::vector<T>& container, const T& element)
{
  return std::find(container.begin(), container.end(), element) != container.end();
}

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;

template <typename Vector>
Vector projection(const Vector& axis, const Vector& v)
{
  Vector axis_normalized = axis.normalized();
  return v.dot(axis_normalized) * axis_normalized;
}

template <typename T>
Matrix3<T> orthonormalize(const Matrix3<T>& mat)
{
  // Preference: z-axis > x-axis > y-axis
  Vector3<T> x, y, z;
  z = mat.col(2);
  x = mat.col(0) - projection(z, mat.col(0).eval());
  y = mat.col(1) - projection(z, mat.col(1).eval()) - projection(x, mat.col(1).eval());
  Matrix3<T> result;
  result << x.normalized(), y.normalized(), z.normalized();
  return result;
}

template <typename PointT>
void pointCloudToEigen(const pcl::PointCloud<PointT>& point_cloud, Eigen::MatrixXd& mat)
{
  mat.resize(point_cloud.size(), 3);
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    mat(i, 0) = point_cloud[i].x;
    mat(i, 1) = point_cloud[i].y;
    mat(i, 2) = point_cloud[i].z;
  }
}

inline void eigenToPointCloud(const Eigen::MatrixXd& mat, pcl::PointCloud<pcl::PointXYZ>& point_cloud)
{
  point_cloud.resize(mat.rows());
  for (std::size_t i = 0; i < mat.rows(); i++)
  {
    point_cloud[i].x = (float)mat(i, 0);
    point_cloud[i].y = (float)mat(i, 1);
    point_cloud[i].z = (float)mat(i, 2);
  }
}

template <typename PointT>
void publishPointCloud(const pcl::PointCloud<PointT>& pcl_cloud, ros::Publisher& publisher, const std::string& frame)
{
  sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(pcl_cloud, *pc2_cloud);
  pc2_cloud->header.frame_id = frame;
  pc2_cloud->header.stamp = ros::Time::now();
  publisher.publish(pc2_cloud);
}

void transformPointCloud(const pcl::PointCloud<pcl::PointNormal>& source_cloud,
                         pcl::PointCloud<pcl::PointXYZ>& target_cloud, const NonrigidTransform& transform);

void transformGrasps(const std::vector<MultiArmGrasp>& source_grasps, std::vector<MultiArmGrasp>& target_grasps,
                     const NonrigidTransform& transform);

}  // namespace utils
}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H
