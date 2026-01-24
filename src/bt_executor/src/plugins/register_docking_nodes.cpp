#include "bt_executor/plugins/register_nodes.hpp"

#include "bt_executor/nodes/docking/docking_phase_update_node.hpp"

namespace bt_executor
{

void registerDockingNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<DockingPhaseUpdateNode>("DockingPhaseUpdate");
}

}  // namespace bt_executor
