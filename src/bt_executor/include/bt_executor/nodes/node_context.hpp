#pragma once

#include <memory>

#include <behaviortree_cpp_v3/tree_node.h>

namespace bt_executor
{

struct BTContext;

std::shared_ptr<BTContext> getContext(const BT::TreeNode &node);
double currentTime(const BT::TreeNode &node);

}  // namespace bt_executor
