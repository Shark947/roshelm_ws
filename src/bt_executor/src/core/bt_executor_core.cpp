#include "bt_executor/core/bt_executor_core.hpp"

#include <exception>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "bt_executor/plugins/register_nodes.hpp"

namespace bt_executor
{

bool BTExecutorCore::init(const std::string &xml_path, const std::vector<std::string> &plugins,
                          const std::shared_ptr<BTContext> &ctx)
{
  blackboard_ = BT::Blackboard::create();
  blackboard_->set("ctx", ctx);
  blackboard_->set("current_time", 0.0);

  const std::vector<std::string> plugin_list =
      plugins.empty() ? std::vector<std::string>{"core", "helm", "docking"} : plugins;

  const std::unordered_map<std::string, std::function<void(BT::BehaviorTreeFactory &)>> registrars{
      {"core", registerCoreNodes},
      {"helm", registerHelmNodes},
      {"docking", registerDockingNodes},
  };

  for (const auto &plugin : plugin_list)
  {
    const auto it = registrars.find(plugin);
    if (it == registrars.end())
    {
      return false;
    }
    it->second(factory_);
  }

  try
  {
    tree_ = factory_.createTreeFromFile(xml_path, blackboard_);
  }
  catch (const std::exception &)
  {
    return false;
  }

  return true;
}

BT::NodeStatus BTExecutorCore::tickOnce()
{
  return tree_.tickRoot();
}

}  // namespace bt_executor
