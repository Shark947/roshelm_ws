#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

#include <string>
#include <vector>

#include "ros_behavior_tree/helm_interface.hpp"

namespace ros_behavior_tree
{

class FlagStringCondition : public BT::ConditionNode
{
public:
  FlagStringCondition(const std::string &name,
                      const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::vector<std::string> splitValues(const std::string &value) const;

  HelmInterface *helm_interface_{nullptr};
};

}  // namespace ros_behavior_tree
