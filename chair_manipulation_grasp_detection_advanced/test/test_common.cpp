#include "test_common.h"
#include <fstream>
#include <ros/package.h>

namespace chair_manipulation
{
std::string loadFileContent(const std::string& filename)
{
  std::ifstream t(filename);
  return std::string((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
}

chair_manipulation::TestParameters::TestParameters()
{
  auto mesh_filename = ros::package::getPath("chair_manipulation_chair_models") + "/models/dining_chair/meshes/"
                                                                                  "dining_chair.ply";
  auto point_cloud_filename = ros::package::getPath("chair_manipulation_chair_models") + "/models/dining_chair/"
                                                                                         "point_clouds/"
                                                                                         "dining_chair.pcd";
  model_ = std::make_shared<Model>(mesh_filename, point_cloud_filename);
  gripper_urdf_ = loadFileContent(ros::package::getPath("chair_manipulation_grasp_detection_advanced") + "/cfg/gripper/"
                                                                                                         "robotiq_2f_"
                                                                                                         "140.urdf");
  gripper_srdf_ = loadFileContent(ros::package::getPath("chair_manipulation_grasp_detection_advanced") + "/cfg/gripper/"
                                                                                                         "robotiq_2f_"
                                                                                                         "140.srdf");
  grasp_database_filename_ = ros::package::getPath("chair_manipulation_grasp_detection_advanced") + "/test/"
                                                                                                    "test_grasp_"
                                                                                                    "database.yaml";

  gripper_params_.base_frame_ = "robotiq_arg2f_base_link";
  gripper_params_.tcp_frame_ = "tcp";
  gripper_params_.contact_threshold_ = 0.001;
  gripper_params_.touch_links_ = { "left_inner_finger_pad", "right_inner_finger_pad" };
  FingerGroup left_finger_group;
  left_finger_group.group_name_ = "left_finger";
  left_finger_group.open_group_state_name_ = "left_finger_open";
  left_finger_group.closed_group_state_name_ = "left_finger_closed";
  FingerGroup right_finger_group;
  right_finger_group.group_name_ = "right_finger";
  right_finger_group.open_group_state_name_ = "right_finger_open";
  right_finger_group.closed_group_state_name_ = "right_finger_closed";
  gripper_params_.finger_groups_ = { left_finger_group, right_finger_group };

  grasp_sampler_params_.max_antipodal_normal_angle_ = 0.1;
  grasp_sampler_params_.max_antipodal_position_angle_ = 0.1;
  grasp_sampler_params_.max_palm_normal_angle_ = 0.1;
  grasp_sampler_params_.gripper_pad_distance_ = 0.1;
  grasp_sampler_params_.gripper_pad_length_ = 0.3;

  for (const auto& pair : grasp_quality_weights_.values_)
  {
    const auto& key = pair.first;
    grasp_quality_weights_.values_[key] = 1.;
  }

  grasp_synthesizer_params_.num_arms_ = 2;
  grasp_synthesizer_params_.friction_coefficient_ = 0.8;
  grasp_synthesizer_params_.num_friction_edges_ = 8;
  grasp_synthesizer_params_.max_arm_radius_ = 1.0;
  grasp_synthesizer_params_.max_yaw_angle_ = M_PI_2;
  grasp_synthesizer_params_.min_height_ = 0.2;
  grasp_synthesizer_params_.nms_distance_threshold_ = 0.1;
  grasp_synthesizer_params_.nms_orientation_threshold_ = 0.1;
  grasp_synthesizer_params_.world_frame_ = "world";
  grasp_synthesizer_params_.arm_base_frames_ = { "robot1_base_link", "robot2_base_link" };

  grasp_database_creator_params_.num_sample_trials_per_model_ = 100;
  grasp_database_creator_params_.min_num_grasps_per_model_ = 3;
  grasp_database_creator_params_.max_num_grasps_per_model_ = 10;
  grasp_database_creator_params_.mesh_filenames_ = { mesh_filename };
  grasp_database_creator_params_.point_cloud_filenames_ = { point_cloud_filename };

  point_cloud_receiver_params_.topics_ = { "/camera1/kinect/depth/points", "/camera2/kinect/depth/points" };
  point_cloud_receiver_params_.world_frame_ = "world";

  point_cloud_preprocessor_params_.voxel_leaf_size_ = 0.03;
  point_cloud_preprocessor_params_.mean_k_ = 50;
  point_cloud_preprocessor_params_.stddev_mul_threshold_ = 3.;
  point_cloud_preprocessor_params_.normal_search_radius_ = 0.05;

  point_cloud_segmentation_params_.plane_max_iterations_ = 100;
  point_cloud_segmentation_params_.plane_distance_threshold_ = 0.03;
  point_cloud_segmentation_params_.cluster_tolerance_ = 0.1;
  point_cloud_segmentation_params_.min_cluster_size_ = 250;
  point_cloud_segmentation_params_.max_cluster_size_ = std::numeric_limits<int>::max();

  point_cloud_registration_params_.lambda_ = cpd::DEFAULT_LAMBDA;
  point_cloud_registration_params_.beta_ = cpd::DEFAULT_BETA;
  point_cloud_registration_params_.max_iterations_ = 150;
  point_cloud_registration_params_.pre_voxel_grid_leaf_size_ = 0.04;
  point_cloud_registration_params_.post_voxel_grid_leaf_size_ = 0.01;
  point_cloud_registration_params_.normal_search_radius_ = 0.03;
  point_cloud_registration_params_.basis_scale_ = 0.1;

  mesh_reconstruction_params_.method_ = "greedy";
  mesh_reconstruction_params_.greedy_search_radius_ = 0.05;
  mesh_reconstruction_params_.greedy_relative_max_distance_ = 2.5;
  mesh_reconstruction_params_.greedy_max_nearest_neighbors_ = 100;
  mesh_reconstruction_params_.greedy_max_surface_angle_ = M_PI / 8;
  mesh_reconstruction_params_.greedy_min_angle_ = M_PI / 18;
  mesh_reconstruction_params_.greedy_max_angle_ = 2 * M_PI / 3;
  mesh_reconstruction_params_.greedy_normal_consistency_ = true;
  mesh_reconstruction_params_.delauny_alpha_ = 0.2;
  mesh_reconstruction_params_.delauny_tolerance_ = 0.01;
  mesh_reconstruction_params_.delauny_offset_ = 0.75;
}

}  // namespace chair_manipulation
