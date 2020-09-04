#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_NONRIGID_TRANSFORM_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_NONRIGID_TRANSFORM_H

#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>

namespace chair_manipulation
{
class NonrigidTransform
{
public:
  using PointCloud = Eigen::MatrixXd;
  using PointCloudConstPtr = std::shared_ptr<const PointCloud>;
  using DeformationField = Eigen::MatrixXd;
  using DeformationFieldConstPtr = std::shared_ptr<const DeformationField>;
  using SearchMethod = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, 3>;
  using SearchMethodPtr = std::shared_ptr<SearchMethod>;

  NonrigidTransform() = default;

  NonrigidTransform(const PointCloudConstPtr& source_points, const DeformationFieldConstPtr& w, double beta = 1.,
                    std::size_t k = 10, double basis_scale = 0.1);

  Eigen::Vector3d operator*(const Eigen::Vector3d& q) const;

  Eigen::Isometry3d operator*(const Eigen::Isometry3d& transform) const;

private:
  PointCloudConstPtr source_points_;
  DeformationFieldConstPtr w_;
  Eigen::MatrixXd g_;
  double beta_{ 0. };
  std::size_t k_{ 0 };
  double basis_scale_;
  std::shared_ptr<SearchMethod> search_method_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_NONRIGID_TRANSFORM_H
