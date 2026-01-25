#include "ros_behavior_tree/bt/register_nodes.hpp"

#include "ros_behavior_tree/nodes/actions/dummy_action.hpp"
#include "ros_behavior_tree/nodes/conditions/common/deploy_triggered.hpp"

namespace ros_behavior_tree
{

void RegisterCommonActions(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<ros_behavior_tree::DummyAction>("DummyAction");
}

void RegisterCommonConditions(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<ros_behavior_tree::DeployTriggered>(
      "DeployTriggered");
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
  RegisterCommonConditions(factory);
  RegisterAlphaConditions(factory);
  RegisterDockingConditions(factory);
}

}  // namespace ros_behavior_tree
