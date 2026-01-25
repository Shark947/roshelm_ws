#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>

namespace ros_behavior_tree
{

void RegisterCommonActions(BT::BehaviorTreeFactory &factory);
void RegisterCommonConditions(BT::BehaviorTreeFactory &factory);
void RegisterAlphaConditions(BT::BehaviorTreeFactory &factory);
void RegisterDockingConditions(BT::BehaviorTreeFactory &factory);

void RegisterAllNodes(BT::BehaviorTreeFactory &factory);

}  // namespace ros_behavior_tree
