#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_TEST_COMMON_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_TEST_COMMON_H

#include "chair_manipulation_grasp_detection_advanced/gripper.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_receiver.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_segmentation.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h"

namespace chair_manipulation
{
struct TestParameters
{
  TestParameters();

  ModelConstPtr model_;
  std::string gripper_urdf_;
  std::string gripper_srdf_;
  std::string grasp_database_filename_;
  GripperParameters gripper_params_;
  GraspSamplerParameters grasp_sampler_params_;
  GraspQualityWeights grasp_quality_weights_;
  GraspSynthesizerParameters grasp_synthesizer_params_;
  GraspDatabaseCreatorParameters grasp_database_creator_params_;

  PointCloudReceiverParameters point_cloud_receiver_params_;
  PointCloudPreprocessorParameters point_cloud_preprocessor_params_;
  PointCloudSegmentationParameters point_cloud_segmentation_params_;
  PointCloudRegistrationParameters point_cloud_registration_params_;
  MeshReconstructionParameters mesh_reconstruction_params_;
};

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_TEST_COMMON_H
