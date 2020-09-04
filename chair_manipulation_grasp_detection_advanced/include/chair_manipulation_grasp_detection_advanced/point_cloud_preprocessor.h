#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_PREPROCESSOR_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_PREPROCESSOR_H

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <ros/ros.h>

namespace chair_manipulation
{
struct PointCloudPreprocessorParameters
{
  void load(ros::NodeHandle& nh);

  double voxel_leaf_size_;
  int mean_k_;
  double stddev_mul_threshold_;
  double normal_search_radius_;
};

class PointCloudPreprocessor
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;
  using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
  using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
  using SurfaceNormalsPtr = SurfaceNormals::Ptr;
  using SearchMethod = pcl::search::KdTree<pcl::PointXYZ>;
  using SearchMethodPtr = SearchMethod::Ptr;

  explicit PointCloudPreprocessor(PointCloudPreprocessorParameters params);

  void setInputCloud(const PointCloudConstPtr& input);

  void preprocess(PointCloud& output);

  void estimateNormals(PointNormalCloud& output);

private:
  PointCloudPreprocessorParameters params_;
  PointCloudConstPtr input_;
  SearchMethodPtr search_method_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_PREPROCESSOR_H
