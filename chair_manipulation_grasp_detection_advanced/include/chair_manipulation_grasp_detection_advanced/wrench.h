#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_WRENCH_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_WRENCH_H

#include "contact.h"
#include "model.h"
#include <vector>

namespace chair_manipulation
{
struct Wrench
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector6d = Eigen::Matrix<double, 6, 1>;

  Wrench() = default;

  Wrench(const Eigen::Vector3d& force, const Eigen::Vector3d& torque)
  {
    value_.block<3, 1>(0, 0) = force;
    value_.block<3, 1>(3, 0) = torque;
  }

  Eigen::Vector3d getForce() const
  {
    return value_.block<3, 1>(0, 0);
  }

  Eigen::Vector3d getTorque() const
  {
    return value_.block<3, 1>(3, 0);
  }

  Vector6d value_ = Vector6d::Zero();
};

struct Hyperplane
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector6d = Eigen::Matrix<double, 6, 1>;

  Hyperplane() = default;

  Hyperplane(const Vector6d& coeffs, double offset) : coeffs_(coeffs), offset_(offset)
  {
  }

  Vector6d coeffs_{ Vector6d::Zero() };
  double offset_{ 0. };
};

class WrenchSpace
{
public:
  explicit WrenchSpace(const std::vector<Contact>& contacts, const Model& model, double friction_coefficient,
                       std::size_t num_friction_edges);

  const std::vector<Wrench>& getWrenches() const
  {
    return wrenches;
  }

  double getFrictionCoefficient() const
  {
    return friction_coefficient_;
  }

  std::size_t getNumFrictionEdges() const
  {
    return num_friction_edges_;
  }

  const std::vector<Hyperplane>& getHyperplanes() const
  {
    return hyperplanes_;
  }

  bool isForceClosure() const
  {
    return force_closure_;
  }

  double getEpsilon1Quality() const
  {
    return epsilon1_quality_;
  }

  double getV1Quality() const
  {
    return v1_quality_;
  }

private:
  std::vector<Wrench> wrenches;
  double friction_coefficient_;
  std::size_t num_friction_edges_;
  std::vector<Hyperplane> hyperplanes_;
  bool force_closure_{ false };
  double epsilon1_quality_{ 0. };
  double v1_quality_{ 0. };

  void addContactWrenches(const Contact& contact, const Model& model);

  void computeConvexHull(std::size_t dim = 6);
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_WRENCH_H
