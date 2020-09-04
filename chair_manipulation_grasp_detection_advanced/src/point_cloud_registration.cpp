#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"

namespace chair_manipulation
{
void PointCloudRegistrationParameters::load(ros::NodeHandle& nh)
{
  lambda_ = nh.param<double>("lambda", cpd::DEFAULT_LAMBDA);
  beta_ = nh.param<double>("beta", cpd::DEFAULT_BETA);
  max_iterations_ = nh.param<int>("max_iterations", cpd::DEFAULT_MAX_ITERATIONS);
  pre_voxel_grid_leaf_size_ = nh.param<double>("pre_voxel_grid_leaf_size", 0.04);
  post_voxel_grid_leaf_size_ = nh.param<double>("post_voxel_grid_leaf_size", 0.02);
  normal_search_radius_ = nh.param<double>("normal_search_radius", 0.05);
  basis_scale_ = nh.param<double>("basis_scale", 0.1);
}

PointCloudRegistration::PointCloudRegistration(PointCloudRegistrationParameters params) : params_(std::move(params))
{
  pre_voxel_filter_.setLeafSize(params_.pre_voxel_grid_leaf_size_, params_.pre_voxel_grid_leaf_size_,
                                params_.pre_voxel_grid_leaf_size_);
  post_voxel_filter_.setLeafSize(params_.post_voxel_grid_leaf_size_, params_.post_voxel_grid_leaf_size_,
                                 params_.post_voxel_grid_leaf_size_);
  search_method_ = SearchMethodPtr{ new SearchMethod };
  normal_estimation_.setRadiusSearch(params_.normal_search_radius_);
  normal_estimation_.setSearchMethod(search_method_);
}

void PointCloudRegistration::setInputSource(const PointNormalCloudConstPtr& source_cloud)
{
  source_cloud_ = source_cloud;
}

void PointCloudRegistration::setInputTarget(const PointNormalCloudConstPtr& target_cloud)
{
  target_cloud_ = target_cloud;
}

void PointCloudRegistration::align(PointNormalCloud& aligned_cloud, NonrigidTransform& transform)
{
  Stopwatch stopwatch;

  auto filtered_source_cloud = PointNormalCloudPtr{ new PointNormalCloud };
  pre_voxel_filter_.setInputCloud(source_cloud_);
  pre_voxel_filter_.filter(*filtered_source_cloud);

  auto source_eigen_cloud = std::make_shared<EigenCloud>();
  auto target_eigen_cloud = std::make_shared<EigenCloud>();
  utils::pointCloudToEigen(*filtered_source_cloud, *source_eigen_cloud);
  utils::pointCloudToEigen(*target_cloud_, *target_eigen_cloud);

  cpd::Nonrigid nonrigid;
  nonrigid.lambda(params_.lambda_);
  nonrigid.beta((params_.beta_));
  nonrigid.max_iterations(params_.max_iterations_);
  nonrigid.normalize(false);

  std::size_t iter = 0;
  nonrigid.add_callback(
      [&](const auto& result) { ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "CPD iteration " << ++iter); });

  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "=== CPD registration ===");
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "Max number of iterations: " << params_.max_iterations_);
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "Number of source cloud points: " << source_eigen_cloud->rows());
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "Number of target cloud points: " << target_eigen_cloud->rows());

  stopwatch.start();
  auto result = nonrigid.run(*target_eigen_cloud, *source_eigen_cloud);
  stopwatch.stop();

  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "CPD finished.");
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "It took" << stopwatch.elapsedSeconds() << "s.");

  auto w = std::make_shared<Eigen::MatrixXd>(result.w);
  transform = NonrigidTransform{ source_eigen_cloud, w, params_.beta_, 10, params_.basis_scale_ };

  auto transformed_cloud = PointCloudPtr{ new PointCloud };
  utils::transformPointCloud(*source_cloud_, *transformed_cloud, transform);

  auto filtered_transformed_cloud = PointCloudPtr{ new PointCloud };
  post_voxel_filter_.setInputCloud(transformed_cloud);
  post_voxel_filter_.filter(*filtered_transformed_cloud);

  auto normals = SurfaceNormalsPtr{ new SurfaceNormals };
  normal_estimation_.setInputCloud(filtered_transformed_cloud);
  normal_estimation_.compute(*normals);

  PointNormalCloud aligned_normal_cloud;
  pcl::concatenateFields(*filtered_transformed_cloud, *normals, aligned_normal_cloud);

  std::vector<int> indices;
  pcl::removeNaNNormalsFromPointCloud(aligned_normal_cloud, aligned_cloud, indices);
}

}  // namespace chair_manipulation
