#include "chair_manipulation_grasp_detection_advanced/point_cloud_segmentation_pipeline.h"
#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_receiver.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_segmentation.h"
#include <chair_manipulation_grasp_detection_advanced/utils.h>
#include <chair_manipulation_grasp_detection_advanced/stopwatch.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

namespace chair_manipulation
{
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointNormalCloudPtr = PointNormalCloud::Ptr;

void runPointCloudSegmentationPipeline()
{
  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{ "~" };
    auto segmented_cloud_topic = nh_priv.param<std::string>("segmented_cloud_topic", "cloud_segmented");
    auto segmented_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(segmented_cloud_topic, 1);

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud receiver.");
    PointCloudReceiverParameters receiver_params;
    ros::NodeHandle receiver_nh{ "~point_cloud_receiver" };
    receiver_params.load(receiver_nh);
    auto world_frame = receiver_params.world_frame_;
    PointCloudReceiver receiver{ std::move(receiver_params) };

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud preprocessor.");
    PointCloudPreprocessorParameters preprocessor_params;
    ros::NodeHandle preprocessor_nh{ "~point_cloud_preprocessor" };
    preprocessor_params.load(preprocessor_nh);
    PointCloudPreprocessor preprocessor{ std::move(preprocessor_params) };

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud segmentation.");
    PointCloudSegmentationParameters segmentation_params;
    ros::NodeHandle segmentation_nh{ "~point_cloud_segmentation" };
    segmentation_params.load(segmentation_nh);
    PointCloudSegmentation segmentation{ std::move(segmentation_params) };

    Stopwatch stopwatch_iteration, stopwatch_step;

    while (ros::ok())
    {
      ROS_DEBUG_STREAM_NAMED("main", "Start next iteration.");
      stopwatch_iteration.start();

      auto received_cloud = PointCloudPtr{ new PointCloud };
      auto preprocessed_cloud = PointCloudPtr{ new PointCloud };
      auto segmented_cloud = PointCloudPtr{ new PointCloud };
      auto point_normal_cloud = PointNormalCloudPtr{ new PointNormalCloud };

      receiver.receive(*received_cloud);

      stopwatch_step.start();
      preprocessor.setInputCloud(received_cloud);
      preprocessor.preprocess(*preprocessed_cloud);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Preprocessing finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      segmentation.setInputCloud(preprocessed_cloud);
      segmentation.segment(*segmented_cloud);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Segmentation finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      preprocessor.setInputCloud(segmented_cloud);
      preprocessor.estimateNormals(*point_normal_cloud);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Normal estimation finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      utils::publishPointCloud(*point_normal_cloud, segmented_cloud_pub, world_frame);

      stopwatch_iteration.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Finished current iteration.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_iteration.elapsedSeconds() << "s.");
    }
  }
  catch (const exception::Runtime& e)
  {
    // Use std::cerr because rosconsole doesn't seem to work consistently here...
    std::cerr << "ERROR: " << e.what();
  }
}

}