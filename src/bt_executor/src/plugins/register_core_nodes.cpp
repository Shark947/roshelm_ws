#include "bt_executor/plugins/register_nodes.hpp"

#include "bt_executor/nodes/core/always_success_condition.hpp"
#include "bt_executor/nodes/core/any_mode_condition.hpp"
#include "bt_executor/nodes/core/deploy_ready_condition.hpp"
#include "bt_executor/nodes/core/flag_condition.hpp"
#include "bt_executor/nodes/core/mode_condition.hpp"
#include "bt_executor/nodes/core/phase_condition.hpp"
#include "bt_executor/nodes/core/set_flag_node.hpp"
#include "bt_executor/nodes/core/set_mode_node.hpp"

namespace bt_executor
{

void registerCoreNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<DeployReadyCondition>("DeployReady");
  factory.registerNodeType<ModeCondition>("ModeIs");
  factory.registerNodeType<AnyModeCondition>("ModeIn");
  factory.registerNodeType<FlagCondition>("FlagIs");
  factory.registerNodeType<PhaseCondition>("PhaseIs");
  factory.registerNodeType<SetModeNode>("SetMode");
  factory.registerNodeType<SetFlagNode>("SetFlag");
  factory.registerNodeType<AlwaysSuccessCondition>("AlwaysSuccessLeaf");
}

}  // namespace bt_executor
