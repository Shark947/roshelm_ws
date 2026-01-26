#include "ros_behavior_tree/bt/register_nodes.hpp"

#include "ros_behavior_tree/nodes/actions/constant_depth_action.hpp"
#include "ros_behavior_tree/nodes/actions/constant_heading_action.hpp"
#include "ros_behavior_tree/nodes/actions/constant_speed_action.hpp"
#include "ros_behavior_tree/nodes/actions/dummy_action.hpp"
#include "ros_behavior_tree/nodes/actions/go_to_depth_action.hpp"
#include "ros_behavior_tree/nodes/actions/mission_complete_action.hpp"
#include "ros_behavior_tree/nodes/actions/station_keep_action.hpp"
#include "ros_behavior_tree/nodes/actions/waypoint_action.hpp"
#include "ros_behavior_tree/nodes/conditions/common/deploy_triggered.hpp"
#include "ros_behavior_tree/nodes/conditions/common/flag_bool_condition.hpp"
#include "ros_behavior_tree/nodes/conditions/common/flag_string_condition.hpp"
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
}

void RegisterDockingActions(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<ros_behavior_tree::GoToDepthAction>(
      "GoToDepthAction");
  factory.registerNodeType<ros_behavior_tree::ConstantDepthAction>(
      "ConstantDepthAction");
  factory.registerNodeType<ros_behavior_tree::ConstantHeadingAction>(
      "ConstantHeadingAction");
  factory.registerNodeType<ros_behavior_tree::StationKeepAction>(
      "StationKeepAction");
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
  factory.registerNodeType<ros_behavior_tree::FlagBoolCondition>(
      "FlagBoolCondition");
  factory.registerNodeType<ros_behavior_tree::FlagStringCondition>(
      "FlagStringCondition");
}

void RegisterAllNodes(BT::BehaviorTreeFactory &factory)
{
  RegisterCommonActions(factory);
  RegisterAlphaActions(factory);
  RegisterDockingActions(factory);
  RegisterCommonConditions(factory);
  RegisterAlphaConditions(factory);
  RegisterDockingConditions(factory);
}

}  // namespace ros_behavior_tree
