#include "chair_manipulation_grasp_detection_advanced/create_grasp_database_pipeline.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <ros/ros.h>
#include <iostream>

namespace chair_manipulation
{
void runCreateGraspDatabasePipeline()
{
  try
  {
    ros::NodeHandle nh_priv{ "~" };
    std::string gripper_urdf, gripper_srdf, grasp_database_filename;
    if (!nh_priv.getParam("gripper_urdf", gripper_urdf))
      throw exception::Parameter{ "Failed to load parameter 'gripper_urdf'." };
    if (!nh_priv.getParam("gripper_srdf", gripper_srdf))
      throw exception::Parameter{ "Failed to load parameter 'gripper_srdf'." };
    if (!nh_priv.getParam("grasp_database_filename", grasp_database_filename))
      throw exception::Parameter{ "Failed to load parameter 'grasp_database_filename'." };

    ROS_DEBUG_STREAM_NAMED("main", "Creating gripper.");
    GripperParameters gripper_params;
    ros::NodeHandle gripper_nh{ "~gripper" };
    gripper_params.load(gripper_nh);
    auto gripper = std::make_shared<Gripper>(std::move(gripper_params), gripper_urdf, gripper_srdf);

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp sampler.");
    GraspSamplerParameters grasp_sampler_params;
    ros::NodeHandle grasp_sampler_nh{ "~grasp_sampler" };
    grasp_sampler_params.load(grasp_sampler_nh);
    auto grasp_sampler = std::make_shared<GraspSampler>(std::move(grasp_sampler_params), gripper);

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp synthesizer.");
    GraspQualityWeights grasp_quality_weights;
    ros::NodeHandle grasp_quality_weights_nh{ "~weights_offline" };
    grasp_quality_weights.load(grasp_quality_weights_nh);

    GraspSynthesizerParameters grasp_synthesizer_params;
    ros::NodeHandle grasp_synthesizer_nh{ "~grasp_synthesizer" };
    grasp_synthesizer_params.load(grasp_synthesizer_nh);
    auto grasp_synthesizer =
        std::make_shared<GraspSynthesizer>(std::move(grasp_synthesizer_params), std::move(grasp_quality_weights));

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp database creator.");
    GraspDatabaseCreatorParameters grasp_database_creator_params;
    ros::NodeHandle grasp_database_creator_nh{ "~grasp_database_creator" };
    grasp_database_creator_params.load(grasp_database_creator_nh);
    GraspDatabaseCreator creator{ std::move(grasp_database_creator_params), grasp_sampler, grasp_synthesizer };

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp database.");
    GraspDatabase database;
    creator.createGraspDatabase(database);
    ros::NodeHandle database_nh{ "grasp_database" };
    database.store(database_nh);

    ROS_DEBUG_STREAM_NAMED("main", "Saving grasp database.");
    std::ostringstream oss_dump_cmd;
    oss_dump_cmd << "rosparam dump " << grasp_database_filename << " grasp_database";
    auto dump_cmd = oss_dump_cmd.str();
    std::system(dump_cmd.c_str());
  }
  catch (const exception::Runtime& e)
  {
    // Use std::cerr because rosconsole doesn't seem to work consistently here...
    std::cerr << "ERROR: " << e.what();
  }
}

}  // namespace chair_manipulation