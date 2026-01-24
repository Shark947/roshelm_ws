#include "bt_executor/nodes/node_context.hpp"

#include <exception>

#include "bt_executor/bt_context.hpp"
#include "bt_executor/ros_io.hpp"

namespace bt_executor
{

std::shared_ptr<BTContext> getContext(const BT::TreeNode &node)
{
  if (!node.config().blackboard)
  {
    return nullptr;
  }

  try
  {
    return node.config().blackboard->get<std::shared_ptr<BTContext>>("ctx");
  }
  catch (const std::exception &)
  {
    return nullptr;
  }
}

double currentTime(const BT::TreeNode &node)
{
  if (node.config().blackboard)
  {
    try
    {
      return node.config().blackboard->get<double>("current_time");
    }
    catch (const std::exception &)
    {
    }
  }

  const auto ctx = getContext(node);
  if (ctx && ctx->ros_io)
  {
    return ctx->ros_io->now();
  }
  return 0.0;
}

}  // namespace bt_executor
