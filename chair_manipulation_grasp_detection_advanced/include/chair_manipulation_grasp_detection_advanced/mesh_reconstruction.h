#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MESH_RECONSTRUCTION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MESH_RECONSTRUCTION_H

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <geometric_shapes/shapes.h>
#include <vtkDelaunay3D.h>
#include <vtkPolyDataMapper.h>
#include <ros/ros.h>

namespace chair_manipulation
{
struct MeshReconstructionParameters
{
  void load(ros::NodeHandle& nh);

  std::string method_;

  double greedy_search_radius_;
  double greedy_relative_max_distance_;
  int greedy_max_nearest_neighbors_;
  double greedy_max_surface_angle_;
  double greedy_min_angle_;
  double greedy_max_angle_;
  bool greedy_normal_consistency_;

  double delauny_alpha_;
  double delauny_tolerance_;
  double delauny_offset_;
};

class MeshReconstruction
{
public:
  using Mesh = shapes::Mesh;
  using PointT = pcl::PointNormal;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudConstPtr = PointCloud::ConstPtr;
  using SearchMethod = pcl::search::KdTree<PointT>;
  using SearchMethodPtr = SearchMethod::Ptr;

  explicit MeshReconstruction(MeshReconstructionParameters params);

  void setInputCloud(const PointCloudConstPtr& input);

  void reconstruct(Mesh& mesh);

private:
  MeshReconstructionParameters params_;
  PointCloudConstPtr input_;
  pcl::GreedyProjectionTriangulation<PointT> greedy_;
  vtkSmartPointer<vtkDelaunay3D> delauny_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MESH_RECONSTRUCTION_H
