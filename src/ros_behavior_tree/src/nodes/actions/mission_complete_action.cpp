#include "ros_behavior_tree/nodes/actions/mission_complete_action.hpp"

namespace ros_behavior_tree
{

MissionCompleteAction::MissionCompleteAction(const std::string &name,
                                             const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList MissionCompleteAction::providedPorts()
{
  return {BT::InputPort<std::string>("mission_value", "complete")};
}

BT::NodeStatus MissionCompleteAction::tick()
{
  if (!helm_interface_)
    return BT::NodeStatus::FAILURE;

  std::string mission_value = "complete";
  getInput("mission_value", mission_value);
  helm_interface_->publishMissionComplete(mission_value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros_behavior_tree
