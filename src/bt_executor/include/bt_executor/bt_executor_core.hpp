#pragma once

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "bt_executor/bt_context.hpp"

namespace bt_executor
{

class BTExecutorCore
{
public:
  bool init(const std::string &xml_path, const std::vector<std::string> &plugins,
            const std::shared_ptr<BTContext> &ctx);

  BT::NodeStatus tickOnce();

  BT::Blackboard::Ptr blackboard() const { return blackboard_; }

private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
};

}  // namespace bt_executor
