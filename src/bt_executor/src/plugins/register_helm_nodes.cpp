#include "bt_executor/plugins/register_nodes.hpp"

#include "bt_executor/nodes/helm/activate_behavior_node.hpp"
#include "bt_executor/nodes/helm/helm_post_tick_node.hpp"
#include "bt_executor/nodes/helm/helm_pre_tick_node.hpp"

namespace bt_executor
{

void registerHelmNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<ActivateBehaviorNode>("ActivateBehavior");
  factory.registerNodeType<HelmPreTickNode>("HelmPreTick");
  factory.registerNodeType<HelmPostTickNode>("HelmPostTick");
}

}  // namespace bt_executor
