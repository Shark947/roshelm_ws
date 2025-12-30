#include <ros/ros.h>

#include "HelmIvP.h"
#include "RosBridge.h"
#include "RosConfigLoader.h"
#include "RosNavPublisher.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_bridge");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  RosNodeConfig config;
  RosConfigLoader loader(private_nh);
  if (!loader.load(config, DEFAULT_CONFIG_PATH))
  {
    ROS_ERROR("Failed to load ROS Helm configuration");
    return 1;
  }

  HelmIvP helm;
  helm.setAppName(config.node_name);
  helm.setConfigPath(config.config_path);
  helm.setRegisterVariablesPath(config.register_variables_path);

  if (!helm.OnStartUp())
  {
    ROS_ERROR("HelmIvP startup failed");
    return 1;
  }

  RosBridge bridge(nh, private_nh, config);
  if (!bridge.initialize())
  {
    ROS_ERROR("RosBridge initialization failed");
    return 1;
  }

  RosNavPublisher nav_publisher(nh);
  if (!nav_publisher.initialize(config.vehicle_name))
  {
    ROS_ERROR("RosNavPublisher initialization failed");
    return 1;
  }

  ros::Rate rate(config.loop_frequency);
  while (ros::ok())
  {
    bridge.deliverPending(helm);
    helm.Iterate();
    bridge.publishDesired(helm);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
