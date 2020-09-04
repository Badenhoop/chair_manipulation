#include "test_common.h"
#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "wrench_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };
  ros::AsyncSpinner spinner{ 1 };
  tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped msg;
  auto point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  TestParameters params;

  auto contacts_str = nh_priv.param<std::string>("contacts", "0.0395327 0.000233644 -4.144e-19 1 4.65414e-12 "
                                                             "1.59234e-12 0.000415997 0.0250483 0 -1 2.33685e-12 "
                                                             "4.28314e-12 0.383165 5.74431e-21 0.576784 3.43154e-14 -1 "
                                                             "-7.32061e-14 0.331477 0.03 0.580225 -7.24013e-13 1 "
                                                             "-3.01672e-14");
  auto contacts = utils::contactsFromStr(contacts_str);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  double friction_coefficient = params.grasp_synthesizer_params_.friction_coefficient_;
  std::size_t num_friction_edges = params.grasp_synthesizer_params_.num_friction_edges_;
  WrenchSpace wrench_space{ contacts, *params.model_, friction_coefficient, num_friction_edges };
  const auto& wrenches = wrench_space.getWrenches();
  for (std::size_t i = 0; i < wrenches.size(); i++)
  {
    const auto& wrench = wrenches[i];
    const auto& contact = contacts[i / num_friction_edges];
    visual_tools.publishArrow(Eigen::Translation3d{ contact.position_ } * transform::fromXAxis(wrench.getForce()),
                              rvt::GREEN);
    visual_tools.publishArrow(Eigen::Translation3d{ contact.position_ } * transform::fromXAxis(wrench.getTorque()),
                              rvt::BLUE);
  }

  visual_tools.publishSphere(params.model_->getCenterOfGravity(), rvt::RED, rvt::LARGE);
  visual_tools.publishText(Eigen::Translation3d{ Eigen::Vector3d{ 0., 0., 0.05 } } *
                               Eigen::Translation3d{ params.model_->getCenterOfGravity() } *
                               Eigen::Isometry3d::Identity(),
                           "cog", rvt::WHITE, rvt::LARGE);

  ROS_INFO_STREAM("force closure: " << wrench_space.isForceClosure());
  ROS_INFO_STREAM("epsilon1 quality: " << wrench_space.getEpsilon1Quality());
  ROS_INFO_STREAM("v1 quality: " << wrench_space.getV1Quality());

  visual_tools.trigger();

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*params.model_->getPointCloud(), point_cloud_pub, "world");
    rate.sleep();
  }

  return 0;
}