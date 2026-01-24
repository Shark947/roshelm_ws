#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>

namespace bt_executor
{

void registerCoreNodes(BT::BehaviorTreeFactory &factory);
void registerHelmNodes(BT::BehaviorTreeFactory &factory);
void registerDockingNodes(BT::BehaviorTreeFactory &factory);

}  // namespace bt_executor
