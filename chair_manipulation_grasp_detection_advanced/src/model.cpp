#include "chair_manipulation_grasp_detection_advanced/model.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/pca.h>
#include <memory>

namespace chair_manipulation
{
Model::Model(const Model::MeshConstPtr& mesh, const Model::PointCloudConstPtr& point_cloud)
  : mesh_(mesh), point_cloud_(point_cloud)
{
  init();
}

Model::Model(const std::string& mesh_filename, const std::string& point_cloud_filename)
{
  auto loaded_mesh = std::make_shared<Mesh>();
  auto loaded_point_cloud = PointCloudPtr{ new PointCloud };

  // Load mesh
  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFilePLY(mesh_filename, mesh))
  {
    std::ostringstream msg;
    msg << "Failed to load model mesh '" << mesh_filename << "'.";
    throw exception::IO{ msg.str() };
  }
  utils::polygonToShapeMesh(mesh, *loaded_mesh);

  // Load point cloud
  if (pcl::io::loadPCDFile(point_cloud_filename, *loaded_point_cloud) != 0)
  {
    std::ostringstream msg;
    msg << "Failed to load model point cloud '" << point_cloud_filename << "'.";
    throw exception::IO{ msg.str() };
  }

  mesh_ = loaded_mesh;
  point_cloud_ = loaded_point_cloud;

  init();
}

void Model::init()
{
  Eigen::Vector4f cogf4;
  pcl::compute3DCentroid(*point_cloud_, cogf4);
  Eigen::Vector3f cogf = cogf4.head<3>();
  center_of_gravity_ = cogf.cast<double>();

  auto it = std::max_element(point_cloud_->begin(), point_cloud_->end(), [&](const PointT& p1, const PointT& p2) {
    return (p1.getVector3fMap() - cogf).norm() < (p2.getVector3fMap() - cogf).norm();
  });
  max_distance_to_center_of_gravity_ = ((*it).getVector3fMap() - cogf).norm();

  Eigen::Vector4f minf4, maxf4;
  pcl::getMinMax3D(*point_cloud_, minf4, maxf4);
  min_ = minf4.head<3>().cast<double>();
  max_ = maxf4.head<3>().cast<double>();
}

}  // namespace chair_manipulation