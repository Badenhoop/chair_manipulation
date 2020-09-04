#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MODEL_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MODEL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <geometric_shapes/shapes.h>

namespace chair_manipulation
{
class Model
{
public:
  using Mesh = shapes::Mesh;
  using MeshPtr = std::shared_ptr<Mesh>;
  using MeshConstPtr = std::shared_ptr<const Mesh>;
  using PointCloud = pcl::PointCloud<pcl::PointNormal>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;
  using PointT = PointCloud::PointType;

  Model(const MeshConstPtr& mesh, const PointCloudConstPtr& point_cloud);

  Model(const std::string& mesh_filename, const std::string& point_cloud_filename);

  const MeshConstPtr& getMesh() const
  {
    return mesh_;
  }

  const PointCloudConstPtr& getPointCloud() const
  {
    return point_cloud_;
  }

  const Eigen::Vector3d& getCenterOfGravity() const
  {
    return center_of_gravity_;
  }

  double getMaxDistanceToCenterOfGravity() const
  {
    return max_distance_to_center_of_gravity_;
  }

  Eigen::Vector3d getMin() const
  {
    return min_;
  }

  Eigen::Vector3d getMax() const
  {
    return max_;
  }

private:
  MeshConstPtr mesh_;
  PointCloudConstPtr point_cloud_;
  Eigen::Vector3d center_of_gravity_;
  double max_distance_to_center_of_gravity_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;

  void init();
};

using ModelPtr = std::shared_ptr<Model>;
using ModelConstPtr = std::shared_ptr<const Model>;

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MODEL_H
