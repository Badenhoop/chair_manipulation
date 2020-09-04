#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction_pipeline.h"
#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_matcher.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>

namespace chair_manipulation
{
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointNormalCloudPtr = PointNormalCloud::Ptr;
using PolygonMesh = pcl::PolygonMesh;
using PolygonMeshPtr = PolygonMesh::Ptr;
using ShapeMesh = shapes::Mesh;
using ShapeMeshPtr = std::shared_ptr<ShapeMesh>;

void runMeshReconstructionPipeline()
{
  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{ "~" };
    std::string segmented_cloud_topic, reconstructed_mesh_topic;
    if (!nh_priv.getParam("segmented_cloud_topic", segmented_cloud_topic))
      throw exception::Parameter{ "Failed to load parameter 'segmented_cloud_topic'." };
    if (!nh_priv.getParam("reconstructed_mesh_topic", reconstructed_mesh_topic))
      throw exception::Parameter{ "Failed to load parameter 'reconstructed_mesh_topic'." };

    ROS_DEBUG_STREAM_NAMED("main", "Loading grasp database.");
    auto grasp_database = std::make_shared<GraspDatabase>();
    ros::NodeHandle grasp_database_nh{ "~grasp_database" };
    grasp_database->load(grasp_database_nh);

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud matcher.");
    PointCloudMatcherParameters point_cloud_matcher_params;
    ros::NodeHandle point_cloud_matcher_nh{ "~point_cloud_matcher" };
    point_cloud_matcher_params.load(point_cloud_matcher_nh);
    PointCloudMatcher point_cloud_matcher{ std::move(point_cloud_matcher_params) };

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud registration.");
    PointCloudRegistrationParameters point_cloud_registration_params;
    ros::NodeHandle point_cloud_registration_nh{ "~point_cloud_registration" };
    point_cloud_registration_params.load(point_cloud_registration_nh);
    PointCloudRegistration point_cloud_registration{ std::move(point_cloud_registration_params) };

    ROS_DEBUG_STREAM_NAMED("main", "Creating mesh reconstruction.");
    MeshReconstructionParameters mesh_reconstruction_params;
    ros::NodeHandle mesh_reconstruction_nh{ "~mesh_reconstruction" };
    mesh_reconstruction_params.load(mesh_reconstruction_nh);
    MeshReconstruction mesh_reconstruction{ std::move(mesh_reconstruction_params) };

    auto reconstructed_mesh_pub = nh.advertise<shape_msgs::Mesh>(reconstructed_mesh_topic, 1);
    Stopwatch stopwatch_step;

    auto segmented_point_cloud2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(segmented_cloud_topic, nh);
    auto segmented_point_cloud = PointNormalCloudPtr{ new PointNormalCloud };
    pcl::fromROSMsg(*segmented_point_cloud2, *segmented_point_cloud);
    ROS_DEBUG_STREAM_NAMED("main", "Received and converted point cloud.");
    ROS_DEBUG_STREAM_NAMED("main", "It has " << segmented_point_cloud->size() << " points.");

    stopwatch_step.start();
    GraspDatabaseElementConstPtr matched_element;
    point_cloud_matcher.setInputCloud(segmented_point_cloud);
    point_cloud_matcher.setInputDatabase(grasp_database);
    point_cloud_matcher.match(matched_element);
    const auto& matched_cloud = matched_element->model_->getPointCloud();
    stopwatch_step.stop();
    ROS_DEBUG_STREAM_NAMED("main", "Matching finished.");
    ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

    stopwatch_step.start();
    auto registered_cloud = PointNormalCloudPtr{ new PointNormalCloud };
    NonrigidTransform nonrigid_transform;
    point_cloud_registration.setInputSource(matched_cloud);
    point_cloud_registration.setInputTarget(segmented_point_cloud);
    point_cloud_registration.align(*registered_cloud, nonrigid_transform);
    stopwatch_step.stop();
    ROS_DEBUG_STREAM_NAMED("main", "Registration finished.");
    ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

    stopwatch_step.start();
    auto shape_mesh = std::make_shared<ShapeMesh>();
    mesh_reconstruction.setInputCloud(registered_cloud);
    mesh_reconstruction.reconstruct(*shape_mesh);
    stopwatch_step.stop();
    ROS_DEBUG_STREAM_NAMED("main", "Mesh reconstruction finished.");
    ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

    stopwatch_step.start();
    shape_msgs::Mesh mesh_msg;
    utils::shapeMeshToMsg(*shape_mesh, mesh_msg);
    reconstructed_mesh_pub.publish(mesh_msg);
    stopwatch_step.stop();
    ROS_DEBUG_STREAM_NAMED("main", "Converted and published reconstructed mesh.");
    ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");
  }
  catch (const exception::Runtime& e)
  {
    std::cerr << "ERROR: " << e.what();
  }
}

}