#include <ros/ros.h>

#include "bt_executor/bt_executor.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bt_executor_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bt_executor::BTExecutor executor(nh, private_nh);
  if (!executor.initialize())
  {
    ROS_ERROR("[bt_executor] Failed to initialize");
    return 1;
  }

  ros::spin();
  return 0;
}
