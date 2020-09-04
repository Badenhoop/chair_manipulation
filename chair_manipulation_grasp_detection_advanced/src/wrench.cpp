#include "chair_manipulation_grasp_detection_advanced/wrench.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/qhull_mutex.h"
#include <ros/ros.h>

extern "C" {
#ifdef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_HAVE_QHULL_2011
#include <libqhull/libqhull.h>
#include <libqhull/mem.h>
#include <libqhull/qset.h>
#include <libqhull/geom.h>
#include <libqhull/merge.h>
#include <libqhull/poly.h>
#include <libqhull/io.h>
#include <libqhull/stat.h>
#else
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
#endif
}

namespace chair_manipulation
{
WrenchSpace::WrenchSpace(const std::vector<Contact>& contacts, const Model& model, double friction_coefficient,
                         std::size_t num_friction_edges)
  : friction_coefficient_(friction_coefficient), num_friction_edges_(num_friction_edges)
{
  for (const auto& contact : contacts)
    addContactWrenches(contact, model);

  computeConvexHull();
}

void WrenchSpace::addContactWrenches(const Contact& contact, const Model& model)
{
  const auto& normal = contact.normal_;
  Eigen::Isometry3d frame = transform::fromZAxis(normal);
  Eigen::Vector3d tangent = frame * Eigen::Vector3d::UnitX();
  Eigen::Vector3d radius = contact.position_ - model.getCenterOfGravity();
  for (std::size_t i = 0; i < num_friction_edges_; i++)
  {
    double angle = i * 2. * M_PI / num_friction_edges_;
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd{ angle, normal };
    Eigen::Vector3d tangent_rotated = q * tangent;
    Eigen::Vector3d force = normal + friction_coefficient_ * tangent_rotated;
    Eigen::Vector3d torque = radius.cross(force) / model.getMaxDistanceToCenterOfGravity();
    wrenches.emplace_back(force, torque);
  }
}

void WrenchSpace::computeConvexHull(std::size_t dim)
{
  std::lock_guard<std::mutex> guard{ qhull_mutex };

  // Copy wrench data in C-style array in order for qhull to process
  auto num_wrenches = wrenches.size();
  auto wrenches_coords = new coordT[num_wrenches * dim];
  for (std::size_t i = 0; i < num_wrenches; i++)
  {
    const auto& wrench = wrenches[i];
    for (std::size_t j = 0; j < dim; j++)
      wrenches_coords[dim * i + j] = wrench.value_[j];
  }

  // This calls qhull to computes the convex hull
  // n: normals with offset
  // FA: report total area and volume
  char qhull_cmd[] = "qhull n FA";
  static FILE* null = fopen("/dev/null", "w");
  int exitcode = qh_new_qhull(dim, num_wrenches, wrenches_coords, true, qhull_cmd, null, null);
  if (exitcode != 0)
  {
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(&curlong, &totlong);
    ROS_WARN_STREAM_NAMED("wrench_space", "Convex hull creation failed.");
    return;
  }
  int num_facets = qh num_facets;
  hyperplanes_.resize(num_facets);

  double max_offset = -std::numeric_limits<double>::max();
  std::size_t i = 0;
  facetT* facet;
  FORALLfacets
  {
    auto& hyperplane = hyperplanes_[i];
    hyperplane.offset_ = facet->offset;
    for (std::size_t j = 0; j < dim; j++)
      hyperplane.coeffs_[j] = facet->normal[j];

    max_offset = std::max(max_offset, hyperplane.offset_);
    i++;
  }

  // We have force closure if there doesn't exist a positive offset
  force_closure_ = max_offset < 0.;
  if (force_closure_)
  {
    // The epsilon1 quality is smallest offset (in magnitude)
    epsilon1_quality_ = -max_offset;
    // The v1 quality is simply the volume of the convex hull
    v1_quality_ = qh totvol;
  }

  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
}

}  // namespace chair_manipulation
