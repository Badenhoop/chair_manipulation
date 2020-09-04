#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_SEGMENTATION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_SEGMENTATION_H

#include <ros/ros.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace chair_manipulation
{
struct PointCloudSegmentationParameters
{
  void load(ros::NodeHandle& nh);

  int plane_max_iterations_;
  double plane_distance_threshold_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
};

class PointCloudSegmentation
{
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;
  using PointIndices = pcl::PointIndices;
  using PointIndicesPtr = PointIndices::Ptr;
  using ModelCoefficients = pcl::ModelCoefficients;
  using ModelCoefficientsPtr = ModelCoefficients::Ptr;
  using SearchMethod = pcl::search::KdTree<PointT>;
  using SearchMethodPtr = SearchMethod::Ptr;

  explicit PointCloudSegmentation(PointCloudSegmentationParameters params);

  void setInputCloud(const PointCloudConstPtr& input);

  void segment(PointCloud& output);

private:
  PointCloudSegmentationParameters params_;
  PointCloudConstPtr input_;
  pcl::SACSegmentation<PointT> plane_segmentation_;
  SearchMethodPtr search_method_;
  pcl::EuclideanClusterExtraction<PointT> clustering_;
};

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_SEGMENTATION_H