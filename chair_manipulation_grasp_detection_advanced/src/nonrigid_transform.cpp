#include "chair_manipulation_grasp_detection_advanced/nonrigid_transform.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <cpd/utils.hpp>

namespace chair_manipulation
{
NonrigidTransform::NonrigidTransform(const PointCloudConstPtr& source_points, const DeformationFieldConstPtr& w,
                                     double beta, std::size_t k, double basis_scale)
  : source_points_(source_points)
  , w_(w)
  , g_(cpd::affinity(*source_points, *source_points, beta))
  , beta_(beta)
  , k_(std::min(k, (std::size_t)source_points->rows()))
  , basis_scale_(basis_scale)
{
  // These two have to be shared pointers in order to make the class copyable.
  // The problem is that the nanoflann KDTree doesn't copy but takes a std::reference_wrapper to the points.
  search_method_ = std::make_shared<SearchMethod>(3, *source_points_);
  search_method_->index->buildIndex();
}

Eigen::Vector3d NonrigidTransform::operator*(const Eigen::Vector3d& q) const
{
  // Identity
  if (k_ == 0)
    return q;

  // Remember: T(p_i) = p_i + v(p_i)
  // where p_i is the i-th point in the source cloud and v(p_i) = g_i * w.
  // For a new point q, we have to estimate its transformation by considering its k nearest neighbors.
  // The transformation of q is then a weighted average of the transformation of each q's neighbors
  // which is weighted by the affinity to q.

  // Retrieve k nearest neighbors to the input point
  std::vector<Eigen::MatrixXd::Index> indices(k_);
  std::vector<double> sqr_distances(k_);
  search_method_->query(q.data(), k_, &indices[0], &sqr_distances[0]);

  // Sum of each transformation weighted by affinity
  Eigen::Vector3d v_sum = Eigen::Vector3d::Zero();
  // Sum of affinities
  double v_normalizer = 0.;

  for (std::size_t i = 0; i < k_; i++)
  {
    const Eigen::Vector3d& p_i = source_points_->row(indices[i]);
    const Eigen::MatrixXd& g_i = g_.row(indices[i]);
    // Compute the affinity between th                                                                              e
    // input point and the current nearest point
    double affinity = cpd::affinity(q.transpose(), p_i.transpose(), beta_)(0, 0);
    // Compute the transformation of the nearest point
    Eigen::Vector3d v = (g_i * *w_).transpose();
    // Weight transform by affinity
    v_sum += affinity * v;
    v_normalizer += affinity;
  }
  return q + (v_sum / v_normalizer);
}

Eigen::Isometry3d NonrigidTransform::operator*(const Eigen::Isometry3d& transform) const
{
  // Transform each basis vector of the rotation matrix individually and orthonormalize
  Eigen::Vector3d t_old = transform.translation();
  Eigen::Vector3d t_new = *this * t_old;
  Eigen::Matrix3d r_old = transform.rotation();
  Eigen::Matrix3d r_new;
  r_new.col(0) = (*this * (basis_scale_ * r_old.col(0) + t_old)) - t_new;
  r_new.col(1) = (*this * (basis_scale_ * r_old.col(1) + t_old)) - t_new;
  r_new.col(2) = (*this * (basis_scale_ * r_old.col(2) + t_old)) - t_new;
  Eigen::Quaterniond q;
  q = utils::orthonormalize(r_new);
  q.normalize();
  Eigen::Isometry3d result;
  result = Eigen::Translation3d{ t_new } * q;
  return result;
}

}  // namespace chair_manipulation