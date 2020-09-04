#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <chair_manipulation_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandAction.h>

namespace chair_manipulation
{
class GraspPlannerActionServer
{
public:
  using CommandAction = chair_manipulation_msgs::GripperCommandAction;
  using CommandGoal = chair_manipulation_msgs::GripperCommandGoal;
  using CommandResult = chair_manipulation_msgs::GripperCommandResult;
  using CommandActionServer = actionlib::SimpleActionServer<CommandAction>;
  using ControlAction = control_msgs::GripperCommandAction;
  using ControlGoal = control_msgs::GripperCommandGoal;
  using ControlResult = control_msgs::GripperCommandResult;
  using ControlActionClient = actionlib::SimpleActionClient<ControlAction>;

  explicit GraspPlannerActionServer(const std::string& action_ns, const std::string& left_finger_action_ns,
                                    const std::string& right_finger_action_ns, double open_joint_position,
                                    double closed_joint_position, double max_effort)
    : nh_priv_("~")
    , command_server_(nh_, action_ns, boost::bind(&GraspPlannerActionServer::callback, this, _1), false)
    , left_finger_client_(left_finger_action_ns, true)
    , right_finger_client_(right_finger_action_ns, true)
    , open_joint_position_(open_joint_position)
    , closed_joint_position_(closed_joint_position)
    , max_effort_(max_effort)
  {
    command_server_.start();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  CommandActionServer command_server_;
  ControlActionClient left_finger_client_;
  ControlActionClient right_finger_client_;
  double open_joint_position_;
  double closed_joint_position_;
  double max_effort_;

  void callback(const CommandGoal::ConstPtr& goal)
  {
    bool success;
    switch (goal->goal)
    {
      case CommandGoal::OPEN:
        sendCommand(open_joint_position_);
        success = true;
        break;

      case CommandGoal::CLOSE:
        sendCommand(closed_joint_position_);
        success = true;
        break;
      default:
        ROS_WARN_STREAM_NAMED("gripper_command_action_server", "Unknown goal.");
        success = false;
        break;
    }

    if (success)
      command_server_.setSucceeded(CommandResult{});
    else
      command_server_.setAborted(CommandResult{});
  }

  void sendCommand(double joint_position)
  {
    ControlGoal control_goal;
    control_goal.command.position = joint_position;
    control_goal.command.max_effort = max_effort_;
    left_finger_client_.sendGoal(control_goal);
    right_finger_client_.sendGoal(control_goal);
    // It turns out that the action client never returns so we just wait some time for the gripper to act
    ros::Duration{3.}.sleep();
    // After that cancel all goals
    left_finger_client_.cancelAllGoals();
    right_finger_client_.cancelAllGoals();
  }
};

}  // namespace chair_manipulation

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gripper_command_action_server_node");

  ros::NodeHandle nh_priv{ "~" };
  auto action_ns = nh_priv.param<std::string>("action_ns", "gripper_command");
  auto left_finger_action_ns = nh_priv.param<std::string>("left_finger_action_ns", "left_finger");
  auto right_finger_action_ns = nh_priv.param<std::string>("right_finger_action_ns", "right_finger");
  auto open_joint_position = nh_priv.param<double>("open_joint_position", 0);
  auto closed_joint_position = nh_priv.param<double>("closed_joint_position", 0.7);
  auto max_effort = nh_priv.param<double>("max_effort", 1000);

  chair_manipulation::GraspPlannerActionServer server{
    action_ns, left_finger_action_ns, right_finger_action_ns, open_joint_position, closed_joint_position, max_effort
  };
  ros::spin();

  return 0;
}
