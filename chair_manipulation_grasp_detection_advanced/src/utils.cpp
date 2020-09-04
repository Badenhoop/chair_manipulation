#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/contact.h"
#include "chair_manipulation_grasp_detection_advanced/nonrigid_transform.h"
#include "chair_manipulation_grasp_detection_advanced/multi_arm_grasp.h"
#include <pcl/io/vtk_lib_io.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometry_msgs/Point.h>
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
namespace utils
{
void polygonToShapeMesh(const pcl::PolygonMesh& polygon_mesh, shapes::Mesh& shape_mesh)
{
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(polygon_mesh, polydata);

  // Make sure that the polygons are triangles
  vtkSmartPointer<vtkTriangleFilter> triangle_filter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangle_filter->SetInputData(polydata);
  triangle_filter->Update();

  polydataToShapeMesh(triangle_filter->GetOutput(), shape_mesh);
}

void polydataToShapeMesh(vtkPolyData* polydata, shapes::Mesh& shape_mesh)
{
  polydata->BuildCells();
  auto points = polydata->GetPoints();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  auto num_vertices = points->GetNumberOfPoints();
  auto num_triangles = cells->GetNumberOfCells();

  // The shapes::Mesh class does not have an assignment operator (which is a shame) so we have to
  // do the memory management by ourselves...
  if (shape_mesh.vertices)
    delete[] shape_mesh.vertices;
  if (shape_mesh.vertex_normals)
    delete[] shape_mesh.vertex_normals;
  if (shape_mesh.triangles)
    delete[] shape_mesh.triangles;
  if (shape_mesh.triangle_normals)
    delete[] shape_mesh.triangle_normals;
  shape_mesh.vertex_count = num_vertices;
  shape_mesh.triangle_count = num_triangles;
  shape_mesh.vertices = new double[num_vertices * 3];
  shape_mesh.vertex_normals = new double[num_vertices * 3];
  shape_mesh.triangles = new unsigned int[num_triangles * 3];
  shape_mesh.triangle_normals = new double[num_triangles * 3];

  // Copy vertices
  for (std::size_t i = 0; i < num_vertices; i++)
  {
    points->GetPoint(i, shape_mesh.vertices + i * 3);
  }

  // Copy triangle indices
  vtkIdType num_points = 0;
  vtkIdType* point_ids = nullptr;
  std::size_t cell_id = 0;
  for (cells->InitTraversal(); cells->GetNextCell(num_points, point_ids); cell_id++)
  {
    shape_mesh.triangles[cell_id * 3 + 0] = point_ids[0];
    shape_mesh.triangles[cell_id * 3 + 1] = point_ids[1];
    shape_mesh.triangles[cell_id * 3 + 2] = point_ids[2];
  }

  // Compute normals
  shape_mesh.computeTriangleNormals();
  shape_mesh.computeVertexNormals();
}

void shapeMeshToMsg(const shapes::Mesh& shape_mesh, shape_msgs::Mesh& msg)
{
  // Copy vertices
  for (std::size_t i = 0; i < shape_mesh.vertex_count; i++)
  {
    geometry_msgs::Point point;
    point.x = shape_mesh.vertices[i * 3 + 0];
    point.y = shape_mesh.vertices[i * 3 + 1];
    point.z = shape_mesh.vertices[i * 3 + 2];
    msg.vertices.push_back(point);
  }

  // Copy triangles
  for (std::size_t i = 0; i < shape_mesh.triangle_count; i++)
  {
    shape_msgs::MeshTriangle triangle;
    triangle.vertex_indices[0] = shape_mesh.triangles[i * 3 + 0];
    triangle.vertex_indices[1] = shape_mesh.triangles[i * 3 + 1];
    triangle.vertex_indices[2] = shape_mesh.triangles[i * 3 + 2];
    msg.triangles.push_back(triangle);
  }
}

void polygonMeshToMsg(const pcl::PolygonMesh& polygon_mesh, shape_msgs::Mesh& msg)
{
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(polygon_mesh, polydata);

  // Make sure that the polygons are triangles
  vtkSmartPointer<vtkTriangleFilter> triangle_filter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangle_filter->SetInputData(polydata);
  triangle_filter->Update();

  vtkSmartPointer<vtkPolyDataMapper> triangle_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangle_mapper->SetInputConnection(triangle_filter->GetOutputPort());
  triangle_mapper->Update();
  polydata = triangle_mapper->GetInput();

  polydata->BuildCells();
  auto points = polydata->GetPoints();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  auto num_vertices = points->GetNumberOfPoints();
  auto num_triangles = cells->GetNumberOfCells();

  msg.vertices.resize(num_vertices);
  msg.triangles.resize(num_triangles);

  // Copy vertices
  for (std::size_t i = 0; i < num_vertices; i++)
  {
    auto& vertex = msg.vertices[i];
    double data[3];
    points->GetPoint(i, data);
    vertex.x = data[0];
    vertex.y = data[1];
    vertex.z = data[2];
  }

  // Copy triangle indices
  vtkIdType num_points = 0;
  vtkIdType* point_ids = nullptr;
  std::size_t cell_id = 0;
  for (cells->InitTraversal(); cells->GetNextCell(num_points, point_ids); cell_id++)
  {
    auto& triangle = msg.triangles[cell_id];
    triangle.vertex_indices[0] = point_ids[0];
    triangle.vertex_indices[1] = point_ids[1];
    triangle.vertex_indices[2] = point_ids[2];
  }
}

std::string loadStringParameter(const XmlRpc::XmlRpcValue& value, const std::string& key)
{
  if (!value.hasMember(key))
  {
    std::ostringstream msg;
    msg << "Failed to find load parameter '" << key << "'.";
    throw exception::Parameter{ msg.str() };
  }
  XmlRpc::XmlRpcValue attribute = value[key];
  if (attribute.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    std::ostringstream msg;
    msg << "Parameter '" << key << "' must be of type string.";
    throw exception::Parameter{ msg.str() };
  }
  return (std::string)attribute;
}

double loadDoubleParameter(const XmlRpc::XmlRpcValue& value, const std::string& key)
{
  if (!value.hasMember(key))
  {
    std::ostringstream msg;
    msg << "Failed to find load parameter '" << key << "'.";
    throw exception::Parameter{ msg.str() };
  }
  XmlRpc::XmlRpcValue attribute = value[key];
  if (attribute.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::ostringstream msg;
    msg << "Parameter '" << key << "' must be of type string.";
    throw exception::Parameter{ msg.str() };
  }
  return (double)attribute;
}

std::string poseToStr(const Eigen::Isometry3d& pose)
{
  auto msg = tf2::toMsg(pose);
  std::ostringstream oss;
  oss << msg.position.x << " " << msg.position.y << " " << msg.position.z << " " << msg.orientation.x << " "
      << msg.orientation.y << " " << msg.orientation.z << " " << msg.orientation.w;
  return oss.str();
}

Eigen::Isometry3d poseFromStr(const std::string& str)
{
  std::istringstream iss{ str };
  std::vector<std::string> entries{ std::istream_iterator<std::string>{ iss }, std::istream_iterator<std::string>{} };
  if (entries.size() != 7)
    throw exception::Parameter{ "Invalid pose string." };

  geometry_msgs::Pose msg;
  msg.position.x = std::stod(entries[0]);
  msg.position.y = std::stod(entries[1]);
  msg.position.z = std::stod(entries[2]);
  msg.orientation.x = std::stod(entries[3]);
  msg.orientation.y = std::stod(entries[4]);
  msg.orientation.z = std::stod(entries[5]);
  msg.orientation.w = std::stod(entries[6]);

  Eigen::Isometry3d result;
  tf2::fromMsg(msg, result);
  return result;
}

std::string vectorToStr(const Eigen::Vector3d& vector)
{
  std::ostringstream oss;
  oss << vector.x() << " " << vector.y() << " " << vector.z();
  return oss.str();
}

Eigen::Vector3d vectorFromStr(const std::string& str)
{
  std::istringstream iss{ str };
  std::vector<std::string> entries{ std::istream_iterator<std::string>{ iss }, std::istream_iterator<std::string>{} };
  if (entries.size() != 3)
    throw exception::Parameter{ "Invalid vector string." };

  return Eigen::Vector3d{ std::stod(entries[0]), std::stod(entries[1]), std::stod(entries[2]) };
}

std::string contactsToStr(const std::vector<Contact>& contacts)
{
  if (contacts.empty())
    return std::string{};

  std::ostringstream oss;
  for (std::size_t i = 0; i < contacts.size(); i++)
  {
    const auto& contact = contacts[i];
    oss << contact.position_.x() << " " << contact.position_.y() << " " << contact.position_.z() << " "
        << contact.normal_.x() << " " << contact.normal_.y() << " " << contact.normal_.z();

    if (i < contacts.size() - 1)
      oss << " ";
  }
  return oss.str();
}

std::vector<Contact> contactsFromStr(const std::string& str)
{
  std::istringstream iss{ str };
  std::vector<std::string> numbers{ std::istream_iterator<std::string>{ iss }, std::istream_iterator<std::string>{} };
  if (numbers.size() % 6 != 0)
    throw exception::IllegalArgument{ "Invalid contact string." };

  std::vector<Contact> contacts;
  std::size_t num_contacts = numbers.size() / 6;
  contacts.resize(num_contacts);
  for (std::size_t i = 0; i < num_contacts; i++)
  {
    auto& contact = contacts[i];
    contact.position_.x() = std::stod(numbers[6 * i + 0]);
    contact.position_.y() = std::stod(numbers[6 * i + 1]);
    contact.position_.z() = std::stod(numbers[6 * i + 2]);
    contact.normal_.x() = std::stod(numbers[6 * i + 3]);
    contact.normal_.y() = std::stod(numbers[6 * i + 4]);
    contact.normal_.z() = std::stod(numbers[6 * i + 5]);
  }
  return contacts;
}

void transformPointCloud(const pcl::PointCloud<pcl::PointNormal>& source_cloud,
                         pcl::PointCloud<pcl::PointXYZ>& target_cloud, const NonrigidTransform& transform)
{
  target_cloud.resize(source_cloud.size());
  for (std::size_t i = 0; i < source_cloud.size(); i++)
  {
    const auto& source_point = source_cloud[i];
    auto& target_point = target_cloud[i];

    Eigen::Vector3d source_position = source_point.getVector3fMap().cast<double>();
    Eigen::Vector3d target_position = transform * source_position;
    target_point.getVector3fMap() = target_position.cast<float>();
  }
}

void transformGrasps(const std::vector<MultiArmGrasp>& source_grasps, std::vector<MultiArmGrasp>& target_grasps,
                     const NonrigidTransform& transform)
{
  target_grasps.resize(source_grasps.size());
  for (std::size_t i = 0; i < source_grasps.size(); i++)
  {
    const auto& source_grasp = source_grasps[i];
    auto& target_grasp = target_grasps[i];
    target_grasp.poses_.resize(source_grasp.poses_.size());
    for (std::size_t j = 0; j < source_grasp.poses_.size(); j++)
      target_grasp.poses_[j] = transform * source_grasp.poses_[j];
  }
}

}  // namespace utils
}  // namespace chair_manipulation