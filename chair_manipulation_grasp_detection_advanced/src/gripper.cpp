#include "chair_manipulation_grasp_detection_advanced/gripper.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <urdf_parser/urdf_parser.h>
#include <chair_manipulation_grasp_detection_advanced/utils.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <memory>

namespace chair_manipulation
{
void GripperParameters::load(ros::NodeHandle& nh)
{
  base_frame_ = nh.param<std::string>("base_frame", "robotiq_arg2f_base_link");
  tcp_frame_ = nh.param<std::string>("tcp_frame", "tcp");
  contact_threshold_ = nh.param<double>("contact_threshold", 0.001);

  if (!nh.getParam("touch_links", touch_links_))
    throw exception::Parameter{ "Failed to load parameter 'touch_links'." };

  XmlRpc::XmlRpcValue finger_groups_array;
  if (!nh.getParam("finger_groups", finger_groups_array) ||
      finger_groups_array.getType() != XmlRpc::XmlRpcValue::TypeArray || finger_groups_array.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'finger_groups'." };

  int num_finger_groups = finger_groups_array.size();
  finger_groups_.resize(num_finger_groups);
  for (int i = 0; i < num_finger_groups; i++)
  {
    const auto& item = finger_groups_array[i];
    auto& finger_group = finger_groups_[i];

    finger_group.group_name_ = utils::loadStringParameter(item, "group_name");
    finger_group.open_group_state_name_ = utils::loadStringParameter(item, "open_group_state");
    finger_group.closed_group_state_name_ = utils::loadStringParameter(item, "closed_group_state");
  }
}

Gripper::Gripper(GripperParameters params, const std::string& gripper_urdf, const std::string& gripper_srdf)
  : params_(std::move(params))
{
  // Parse urdf and srdf descriptions
  auto urdf_model = urdf::parseURDF(gripper_urdf);
  auto srdf_model = std::make_shared<srdf::Model>();
  srdf_model->initString(*urdf_model, gripper_srdf);

  // Initialize robot model, robot state and collision environment
  robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);

  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  robot_state_->update();

  acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>();
  // Use default collision operations in the SRDF to setup the acm
  const auto& collision_links = robot_model_->getLinkModelNamesWithCollisionGeometry();
  acm_->setEntry(collision_links, collision_links, false);
  // Allow collisions for pairs that have been disabled
  const auto& disabled_collisions = srdf_model->getDisabledCollisionPairs();
  for (const srdf::Model::DisabledCollision& it : disabled_collisions)
    acm_->setEntry(it.link1_, it.link2_, true);

  cenv_ = std::make_shared<collision_detection::CollisionEnvFCL>(robot_model_);

  // Get tcp to base transform
  bool frame_found;
  Eigen::Isometry3d world_to_base = robot_state_->getFrameTransform(params_.base_frame_, &frame_found);
  if (!frame_found)
    throw exception::Parameter{ "Cannot find frame '" + params_.base_frame_ + "'." };
  Eigen::Isometry3d world_to_tcp = robot_state_->getFrameTransform(params_.tcp_frame_, &frame_found);
  if (!frame_found)
    throw exception::Parameter{ "Cannot find frame '" + params_.tcp_frame_ + "'." };
  tcp_to_base_ = world_to_base * world_to_tcp.inverse();
  base_pose_ = Eigen::Isometry3d::Identity();

  // Load finger group description
  loadGroupStates();
}

void Gripper::setTcpPose(const Eigen::Isometry3d& tcp_pose)
{
  // Get the required orientation of the base to achieve the given gripper pose
  base_pose_ = tcp_pose * tcp_to_base_;
}

void Gripper::addCollisionObject(const shapes::ShapeConstPtr& object, const Eigen::Isometry3d& pose)
{
  cenv_->getWorld()->addToObject(std::to_string(running_id_++), object, pose);
}

void Gripper::clearCollisionObjects()
{
  cenv_->getWorld()->clearObjects();
}

void Gripper::setStateOpen()
{
  for (const auto& finger_group : params_.finger_groups_)
    robot_state_->setVariablePositions(open_group_state_values_[finger_group.group_name_]);
}

bool Gripper::isColliding()
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  updateState();
  cenv_->checkCollision(req, res, *robot_state_, *acm_);
  return res.collision;
}

bool Gripper::grasp(std::vector<Contact>& contacts)
{
  // The gripper must initially be in a collision-free state
  if (isColliding())
    return false;

  // Move each finger group individually to closed position and stop at a point of collision
  for (const auto& finger_group : params_.finger_groups_)
  {
    const auto& open_values = open_group_state_values_[finger_group.group_name_];
    const auto& closed_values = closed_group_state_values_[finger_group.group_name_];
    if (!moveToContacts(open_values, closed_values, finger_group, contacts))
      return false;
  }

  return true;
}

bool Gripper::moveToContacts(const std::map<std::string, double>& open_values,
                             const std::map<std::string, double>& closed_values, const FingerGroup& finger_group,
                             std::vector<Contact>& contacts)
{
  double t = 0.;
  double delta = 1.;
  constexpr double MIN_DELTA = 1.0e-20;
  while (delta > MIN_DELTA)
  {
    delta *= 0.5;

    // Update values
    for (const auto& pair : open_values)
    {
      const auto& variable = pair.first;
      double value = t * closed_values.at(variable) + (1. - t) * open_values.at(variable);
      robot_state_->setVariablePosition(variable, value);
    }
    updateState();

    // Check for self-collision.
    // We treat self-collisions as invalid states, meaning that if only self-collisions appear
    // during the interpolation we will return false.
    collision_detection::CollisionRequest req_self;
    req_self.group_name = finger_group.group_name_;
    collision_detection::CollisionResult res_self;
    cenv_->checkSelfCollision(req_self, res_self, *robot_state_, *acm_);
    if (res_self.collision)
    {
      t -= delta;

      // This means that there is a collision in the initial state which is illegal
      if (t < 0.)
      {
        return false;
      }
    }
    else
    {
      // Check for collision between the robot and the world
      collision_detection::CollisionRequest req_world;
      req_world.group_name = finger_group.group_name_;
      req_world.contacts = true;
      req_world.max_contacts = 1;
      req_world.max_contacts_per_pair = 1;
      collision_detection::CollisionResult res_world;
      cenv_->checkRobotCollision(req_world, res_world, *robot_state_);
      if (res_world.collision)
      {
        t -= delta;

        // This means that there is a collision in the initial state which is illegal
        if (t < 0.)
        {
          return false;
        }

        // A link of the robot is in contact with some body in the world if the distance between them is negative
        // and less or equal to the contact threshold
        bool contact_found = false;
        for (const auto& pair : res_world.contacts)
        {
          const auto& detected_contacts = pair.second;
          for (const auto& detected_contact : detected_contacts)
          {
            if (detected_contact.depth > params_.contact_threshold_)
              continue;

            assert((detected_contact.body_type_1 == collision_detection::BodyType::ROBOT_LINK &&
                    detected_contact.body_type_2 == collision_detection::BodyType::WORLD_OBJECT) ||
                   (detected_contact.body_type_1 == collision_detection::BodyType::WORLD_OBJECT &&
                    detected_contact.body_type_2 == collision_detection::BodyType::ROBOT_LINK));

            // Only allow collisions with specified touch links
            if (detected_contact.body_type_1 == collision_detection::BodyType::ROBOT_LINK &&
                !utils::contains(params_.touch_links_, detected_contact.body_name_1))
              return false;

            if (detected_contact.body_type_2 == collision_detection::BodyType::ROBOT_LINK &&
                !utils::contains(params_.touch_links_, detected_contact.body_name_2))
              return false;

            contact_found = true;
            Contact added_contact;
            added_contact.position_ = detected_contact.pos;

            if (detected_contact.body_type_1 == collision_detection::BodyType::ROBOT_LINK)
              added_contact.normal_ = detected_contact.normal;
            else if (detected_contact.body_type_2 == collision_detection::BodyType::ROBOT_LINK)
              added_contact.normal_ = -detected_contact.normal;

            contacts.push_back(added_contact);
          }
        }
        if (contact_found)
        {
          return true;
        }
      }
      else
      {
        t += delta;
      }
    }
  }
  return false;
}

void Gripper::updateState()
{
  robot_state_->updateStateWithLinkAt(params_.base_frame_, base_pose_);
  robot_state_->updateCollisionBodyTransforms();
}

void Gripper::loadGroupStates()
{
  for (const auto& finger_group : params_.finger_groups_)
  {
    const auto& group_name = finger_group.group_name_;
    const auto& jmg = robot_model_->getJointModelGroup(group_name);

    if (!jmg->getVariableDefaultPositions(finger_group.open_group_state_name_, open_group_state_values_[group_name]))
      throw exception::Parameter{ "Failed to retrieve open group state values." };

    if (!jmg->getVariableDefaultPositions(finger_group.closed_group_state_name_,
                                          closed_group_state_values_[group_name]))
      throw exception::Parameter{ "Failed to retrieve closed group state values." };
  }
}

}  // namespace chair_manipulation