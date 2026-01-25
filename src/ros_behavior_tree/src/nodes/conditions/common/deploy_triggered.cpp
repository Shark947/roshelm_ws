#include "ros_behavior_tree/nodes/conditions/common/deploy_triggered.hpp"

#include <ros/ros.h>

namespace ros_behavior_tree
{

DeployTriggered::DeployTriggered(const std::string &name,
                                 const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("nav_store", store_);
  }
}

BT::PortsList DeployTriggered::providedPorts()
{
  return {};
}

BT::NodeStatus DeployTriggered::tick()
{
  if (!store_)
    return BT::NodeStatus::FAILURE;

  bool value = false;
  if (!store_->preferredDeploy(value, ros::Duration(0.0)))
    return BT::NodeStatus::FAILURE;
  return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace ros_behavior_tree
