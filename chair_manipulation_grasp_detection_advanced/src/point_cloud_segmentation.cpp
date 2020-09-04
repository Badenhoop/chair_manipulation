#include "chair_manipulation_grasp_detection_advanced/point_cloud_segmentation.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <pcl/filters/extract_indices.h>

namespace chair_manipulation
{
void PointCloudSegmentationParameters::load(ros::NodeHandle& nh)
{
  plane_max_iterations_ = nh.param<int>("plane_max_iterations", 100);
  plane_distance_threshold_ = nh.param<double>("plane_distance_threshold", 0.03);
  cluster_tolerance_ = nh.param<double>("cluster_tolerance", 0.1);
  min_cluster_size_ = nh.param<int>("min_cluster_size", 250);
  max_cluster_size_ = nh.param<int>("max_cluster_size", std::numeric_limits<int>::max());
}

PointCloudSegmentation::PointCloudSegmentation(PointCloudSegmentationParameters params) : params_(std::move(params))
{
  plane_segmentation_.setOptimizeCoefficients(true);
  plane_segmentation_.setModelType(pcl::SACMODEL_PLANE);
  plane_segmentation_.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation_.setMaxIterations(params_.plane_max_iterations_);
  plane_segmentation_.setDistanceThreshold(params_.plane_distance_threshold_);
  search_method_ = SearchMethodPtr{ new SearchMethod };
  clustering_.setSearchMethod(search_method_);
  clustering_.setClusterTolerance(params_.cluster_tolerance_);
  clustering_.setMinClusterSize(params_.min_cluster_size_);
  clustering_.setMaxClusterSize(params_.max_cluster_size_);
}

void PointCloudSegmentation::setInputCloud(const PointCloudSegmentation::PointCloudConstPtr& input)
{
  input_ = input;
}

void PointCloudSegmentation::segment(PointCloud& output)
{
  auto inliers = PointIndicesPtr{ new PointIndices };
  auto coefficients = ModelCoefficientsPtr{ new ModelCoefficients };
  plane_segmentation_.setInputCloud(input_);
  plane_segmentation_.segment(*inliers, *coefficients);

  PointCloudConstPtr filtered_cloud = input_;
  if (inliers->indices.empty())
  {
    ROS_WARN_STREAM_NAMED("point_cloud_segmentation", "Could not estimate a planar model for the given point cloud.");
  }
  else
  {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input_);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    PointCloud plane_cloud;
    extract.filter(plane_cloud);

    // Remove the planar inliers, extract the rest
    auto plane_segmented_cloud = PointCloudPtr{ new PointCloud };
    extract.setNegative(true);
    extract.filter(*plane_segmented_cloud);

    filtered_cloud = plane_segmented_cloud;
  }

  std::vector<PointIndices> cluster_indices;
  search_method_->setInputCloud(filtered_cloud);
  clustering_.setInputCloud(filtered_cloud);
  clustering_.extract(cluster_indices);

  if (cluster_indices.empty())
    throw exception::PointCloud{"No clusters found."};

  // Find the indices with the most points
  auto it =
      std::max_element(cluster_indices.begin(), cluster_indices.end(), [&](const auto& indices1, const auto& indices2) {
        return indices1.indices.size() < indices2.indices.size();
      });

  const auto& indices = *it;
  pcl::copyPointCloud(*filtered_cloud, indices, output);
}

}  // namespace chair_manipulation
