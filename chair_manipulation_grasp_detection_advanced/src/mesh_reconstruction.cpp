#include <chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h>
#include <chair_manipulation_grasp_detection_advanced/utils.h>
#include <chair_manipulation_grasp_detection_advanced/exception.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkTriangleFilter.h>

namespace chair_manipulation
{
void MeshReconstructionParameters::load(ros::NodeHandle& nh)
{
  method_ = nh.param<std::string>("method", "greedy");

  greedy_search_radius_ = nh.param<double>("greedy/search_radius", 0.025);
  greedy_relative_max_distance_ = nh.param<double>("greedy/relative_max_distance", 2.5);
  greedy_max_nearest_neighbors_ = nh.param<int>("greedy/max_nearest_neighbors", 100);
  greedy_max_surface_angle_ = nh.param<double>("greedy/max_surface_angle", M_PI / 4);
  greedy_min_angle_ = nh.param<double>("greedy/min_angle", M_PI / 18);
  greedy_max_angle_ = nh.param<double>("greedy/max_angle", 2 * M_PI / 3);
  greedy_normal_consistency_ = nh.param<bool>("greedy/normal_consistency", false);

  delauny_alpha_ = nh.param<double>("delauny/alpha", 0.2);
  delauny_tolerance_ = nh.param<double>("delauny/tolerance", 0.01);
  delauny_offset_ = nh.param<double>("delauny/offset", 0.75);
}

MeshReconstruction::MeshReconstruction(MeshReconstructionParameters params) : params_(std::move(params))
{
  auto search_method = SearchMethodPtr{ new SearchMethod };
  greedy_.setSearchMethod(search_method);
  greedy_.setSearchRadius(params_.greedy_search_radius_);
  greedy_.setMu(params_.greedy_relative_max_distance_);
  greedy_.setMaximumNearestNeighbors(params_.greedy_max_nearest_neighbors_);
  greedy_.setMaximumSurfaceAngle(params_.greedy_max_surface_angle_);
  greedy_.setMinimumAngle(params_.greedy_min_angle_);
  greedy_.setMaximumAngle(params_.greedy_max_angle_);
  greedy_.setNormalConsistency(params_.greedy_normal_consistency_);

  delauny_ = vtkSmartPointer<vtkDelaunay3D>::New();
  delauny_->SetAlpha(params_.delauny_alpha_);
  delauny_->SetTolerance(params_.delauny_tolerance_);
  delauny_->SetOffset(params_.delauny_offset_);
  delauny_->BoundingTriangulationOff();
}

void MeshReconstruction::setInputCloud(const PointCloudConstPtr& input)
{
  input_ = input;
}

void MeshReconstruction::reconstruct(Mesh& mesh)
{
  if (params_.method_ == "greedy")
  {
    pcl::PolygonMesh polygon_mesh;
    greedy_.setInputCloud(input_);
    greedy_.reconstruct(polygon_mesh);
    utils::polygonToShapeMesh(polygon_mesh, mesh);
  }
  else if (params_.method_ == "delauny")
  {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& point : input_->points)
      points->InsertNextPoint(point.x, point.y, point.z);

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points.Get());

    delauny_->SetInputData(polydata);
    delauny_->Update();

    vtkSmartPointer<vtkDataSetSurfaceFilter> surface_filter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
    surface_filter->SetInputConnection(delauny_->GetOutputPort());
    surface_filter->Update();

    vtkSmartPointer<vtkTriangleFilter> triangle_filter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangle_filter->SetInputConnection(surface_filter->GetOutputPort());
    triangle_filter->Update();

    utils::polydataToShapeMesh(triangle_filter->GetOutput(), mesh);
  }
  else
  {
    throw exception::Parameter{"Invalid reconstruction method " + params_.method_ + "."};
  }
}

}  // namespace chair_manipulation
