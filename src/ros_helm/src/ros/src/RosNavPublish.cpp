#include <ros/ros.h>

#include "RosNavPublisher.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_nav_publish");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string vehicle_name;
  if (!private_nh.getParam("vehicle_name", vehicle_name) || vehicle_name.empty())
  {
    ROS_ERROR_STREAM("[RosNavPublish] vehicle_name param missing or empty!");
    return 1;
  }

  RosNavPublisher publisher(nh);
  if (!publisher.initialize(vehicle_name))
    return 1;

  ros::spin();
  return 0;
}
