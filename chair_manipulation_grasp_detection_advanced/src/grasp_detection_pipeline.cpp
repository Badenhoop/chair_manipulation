#include "chair_manipulation_grasp_detection_advanced/grasp_detection_pipeline.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_matcher.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include "chair_manipulation_grasp_detection_advanced/gripper.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h"
#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"
#include "chair_manipulation_grasp_detection_advanced/ik_checker.h"
#include "chair_manipulation_grasp_detection_advanced/robot.h"
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <iostream>

namespace chair_manipulation
{
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointNormalCloudPtr = PointNormalCloud::Ptr;
using EigenCloud = Eigen::MatrixXd;
using EigenCloudPtr = std::shared_ptr<EigenCloud>;
using PolygonMesh = pcl::PolygonMesh;
using PolygonMeshPtr = PolygonMesh::Ptr;
using ShapeMesh = shapes::Mesh;
using ShapeMeshPtr = std::shared_ptr<ShapeMesh>;
using GraspCandidate = GraspSynthesizer::GraspCandidate;

void GraspDetectionParameters::load(ros::NodeHandle& nh)
{
  run_once_ = nh.param<bool>("run_once", false);
  num_sample_trials_per_grasp_ = nh.param<int>("num_sample_trials_per_grasp", 100);
  sample_radius_ = nh.param<double>("sample_radius", 0.1);

  XmlRpc::XmlRpcValue grasp_frame_items;
  if (!nh.getParam("grasp_frames", grasp_frame_items) ||
      grasp_frame_items.getType() != XmlRpc::XmlRpcValue::TypeArray || grasp_frame_items.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'topics'." };

  int num_grasp_frames = grasp_frame_items.size();
  grasp_frames_.resize(num_grasp_frames);
  for (int i = 0; i < num_grasp_frames; i++)
  {
    XmlRpc::XmlRpcValue grasp_frame_item = grasp_frame_items[i];
    grasp_frames_[i] = (std::string)grasp_frame_item;
  }
}

void runGraspDetectionPipeline()
{
  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{ "~" };
    std::string gripper_urdf, gripper_srdf, segmented_cloud_topic, registered_cloud_topic, reconstructed_mesh_topic;
    if (!nh_priv.getParam("gripper_urdf", gripper_urdf))
      throw exception::Parameter{ "Failed to load parameter 'gripper_urdf'." };
    if (!nh_priv.getParam("gripper_srdf", gripper_srdf))
      throw exception::Parameter{ "Failed to load parameter 'gripper_srdf'." };
    if (!nh_priv.getParam("segmented_cloud_topic", segmented_cloud_topic))
      throw exception::Parameter{ "Failed to load parameter 'segmented_cloud_topic'." };
    if (!nh_priv.getParam("reconstructed_mesh_topic", reconstructed_mesh_topic))
      throw exception::Parameter{ "Failed to load parameter 'reconstructed_mesh_topic'." };
    if (!nh_priv.getParam("registered_cloud_topic", registered_cloud_topic))
      throw exception::Parameter{ "Failed to load parameter 'transformed_cloud_topic'." };

    GraspDetectionParameters grasp_detection_params;
    ros::NodeHandle grasp_detection_nh{ "~grasp_detection" };
    grasp_detection_params.load(grasp_detection_nh);

    ROS_DEBUG_STREAM_NAMED("main", "Creating gripper.");
    GripperParameters gripper_params;
    ros::NodeHandle gripper_nh{ "~gripper" };
    gripper_params.load(gripper_nh);
    auto gripper = std::make_shared<Gripper>(std::move(gripper_params), gripper_urdf, gripper_srdf);

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp sampler.");
    GraspSamplerParameters grasp_sampler_params;
    ros::NodeHandle grasp_sampler_nh{ "~grasp_sampler" };
    grasp_sampler_params.load(grasp_sampler_nh);
    GraspSampler grasp_sampler{ std::move(grasp_sampler_params), gripper };

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp synthesizer.");
    GraspQualityWeights grasp_quality_weights;
    ros::NodeHandle grasp_quality_weights_nh{ "~weights_online" };
    grasp_quality_weights.load(grasp_quality_weights_nh);

    GraspSynthesizerParameters grasp_synthesizer_params;
    ros::NodeHandle grasp_synthesizer_nh{ "~grasp_synthesizer" };
    grasp_synthesizer_params.load(grasp_synthesizer_nh);
    GraspSynthesizer grasp_synthesizer{ std::move(grasp_synthesizer_params), std::move(grasp_quality_weights) };

    ROS_DEBUG_STREAM_NAMED("main", "Loading grasp database.");
    auto grasp_database = std::make_shared<GraspDatabase>();
    ros::NodeHandle grasp_database_nh{ "~grasp_database" };
    grasp_database->load(grasp_database_nh);

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud preprocessor.");
    PointCloudPreprocessorParameters preprocessor_params;
    ros::NodeHandle preprocessor_nh{ "~point_cloud_preprocessor" };
    preprocessor_params.load(preprocessor_nh);
    PointCloudPreprocessor preprocessor{ std::move(preprocessor_params) };

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

    ROS_DEBUG_STREAM_NAMED("main", "Creating robot.");
    RobotParameters robot_params;
    ros::NodeHandle robot_nh{"~robot"};
    robot_params.load(robot_nh);
    auto robot = std::make_shared<Robot>(std::move(robot_params));

    ROS_DEBUG_STREAM_NAMED("main", "Creating IK checker.");
    IKCheckerParameters ik_checker_params;
    ros::NodeHandle ik_checker_nh{ "~ik_checker" };
    ik_checker_params.load(ik_checker_nh);
    IKChecker ik_checker{ std::move(ik_checker_params), robot };

    tf2_ros::StaticTransformBroadcaster broadcaster;
    auto reconstructed_mesh_pub = nh.advertise<shape_msgs::Mesh>(reconstructed_mesh_topic, 1);
    auto transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(registered_cloud_topic, 1);
    Stopwatch stopwatch_iteration, stopwatch_step;

    do
    {
      ROS_DEBUG_STREAM_NAMED("main", "Start next iteration.");
      stopwatch_iteration.start();

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
      utils::publishPointCloud(*registered_cloud, transformed_cloud_pub, robot->getWorldFrame());
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Registration finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      const auto& matched_grasps = matched_element->grasps_;
      std::vector<MultiArmGrasp> prior_grasps;
      utils::transformGrasps(matched_grasps, prior_grasps, nonrigid_transform);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Transforming grasps finished.");
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

      stopwatch_step.start();
      Model model{ shape_mesh, registered_cloud };
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Creating model finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      std::vector<GraspHypothesis> hypotheses;
      grasp_sampler.filterCollisionFree(model, prior_grasps, hypotheses);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Filtered collision free grasps.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      std::vector<GraspHypothesis> ik_filtered_hypotheses;
      ik_checker.filter(hypotheses, ik_filtered_hypotheses);
      stopwatch_step.stop();
      if (ik_filtered_hypotheses.size() < 2)
      {
        ROS_WARN_STREAM_NAMED("main", "Failed: Less than two grasp hypotheses after IK filtering.");
        continue;
      }
      ROS_DEBUG_STREAM_NAMED("main", "Filtered valid IK grasps.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      std::vector<MultiArmGrasp> grasps;
      std::vector<GraspCandidate> candidates;
      grasp_synthesizer.generateGraspCandidates(ik_filtered_hypotheses, candidates);
      grasp_synthesizer.synthesize(candidates, model, 1, grasps);
      if (grasps.empty())
      {
        ROS_WARN_STREAM_NAMED("main", "Failed: Unable to find a valid grasp.");
        continue;
      }
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Synthesizing grasps finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      const auto& published_grasp = grasps[0];
      for (std::size_t i = 0; i < published_grasp.poses_.size(); i++)
      {
        const auto& pose = published_grasp.poses_[i];
        const auto& frame = grasp_detection_params.grasp_frames_[i];
        auto msg = tf2::eigenToTransform(pose);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = robot->getWorldFrame();
        msg.child_frame_id = robot->getClosestArm(pose)->getGraspFrame();
        broadcaster.sendTransform(msg);
      }
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Publishing grasp transforms finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_iteration.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Finished current iteration.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_iteration.elapsedSeconds() << "s.");
    } while (!grasp_detection_params.run_once_ && ros::ok());
  }
  catch (const exception::Runtime& e)
  {
    // Use std::cerr because rosconsole doesn't seem to work consistently here...
    std::cerr << "ERROR: " << e.what();
  }
}

}  // namespace chair_manipulation