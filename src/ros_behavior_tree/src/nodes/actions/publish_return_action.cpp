#include "ros_behavior_tree/nodes/actions/publish_return_action.hpp"

namespace ros_behavior_tree
{

PublishReturnAction::PublishReturnAction(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList PublishReturnAction::providedPorts()
{
  return {BT::InputPort<bool>("return_value", true, "Return value to publish")};
}

BT::NodeStatus PublishReturnAction::tick()
{
  if (!helm_interface_)
    return BT::NodeStatus::FAILURE;

  bool return_value = true;
  getInput("return_value", return_value);
  helm_interface_->publishReturn(return_value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros_behavior_tree
