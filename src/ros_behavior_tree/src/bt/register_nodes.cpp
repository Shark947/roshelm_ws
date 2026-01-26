#include "ros_behavior_tree/bt/register_nodes.hpp"

#include "ros_behavior_tree/nodes/actions/constant_speed_action.hpp"
#include "ros_behavior_tree/nodes/actions/dummy_action.hpp"
#include "ros_behavior_tree/nodes/actions/mission_complete_action.hpp"
#include "ros_behavior_tree/nodes/actions/publish_return_action.hpp"
#include "ros_behavior_tree/nodes/actions/waypoint_action.hpp"
#include "ros_behavior_tree/nodes/conditions/common/deploy_triggered.hpp"
#include "ros_behavior_tree/nodes/conditions/common/return_triggered.hpp"
#include "ros_behavior_tree/nodes/conditions/common/speed_triggered.hpp"

namespace ros_behavior_tree
{

void RegisterCommonActions(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<ros_behavior_tree::DummyAction>("DummyAction");
}

void RegisterAlphaActions(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<ros_behavior_tree::WaypointAction>("WaypointAction");
  factory.registerNodeType<ros_behavior_tree::ConstantSpeedAction>(
      "ConstantSpeedAction");
  factory.registerNodeType<ros_behavior_tree::MissionCompleteAction>(
      "MissionCompleteAction");
  factory.registerNodeType<ros_behavior_tree::PublishReturnAction>(
      "PublishReturnAction");
}

void RegisterCommonConditions(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<ros_behavior_tree::DeployTriggered>(
      "DeployTriggered");
  factory.registerNodeType<ros_behavior_tree::ReturnTriggered>(
      "ReturnTriggered");
  factory.registerNodeType<ros_behavior_tree::SpeedTriggered>(
      "SpeedTriggered");
}

void RegisterAlphaConditions(BT::BehaviorTreeFactory &factory)
{
  (void)factory;
}

void RegisterDockingConditions(BT::BehaviorTreeFactory &factory)
{
  (void)factory;
}

void RegisterAllNodes(BT::BehaviorTreeFactory &factory)
{
  RegisterCommonActions(factory);
  RegisterAlphaActions(factory);
  RegisterCommonConditions(factory);
  RegisterAlphaConditions(factory);
  RegisterDockingConditions(factory);
}

}  // namespace ros_behavior_tree
