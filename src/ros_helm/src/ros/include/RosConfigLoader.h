#pragma once

#include <map>
#include <string>
#include <ros/ros.h>

struct RosNodeConfig
{
  std::string node_name{"ros_bridge"};
  std::string config_path;
  std::string register_variables_path;
  std::string vehicle_name{"auv"};
  std::string current_heading_topic;
  std::string current_speed_topic;
  std::string current_depth_topic;
  std::string desired_heading_topic;
  std::string desired_speed_topic;
  std::string desired_depth_topic;
  std::string deploy_topic;
  std::string return_topic;
  std::string frame_id{"map"};
  double loop_frequency{10.0};
  bool deploy_default{false};
  bool return_default{false};
  std::map<std::string, double> nav_defaults;
};

class RosConfigLoader
{
public:
  explicit RosConfigLoader(ros::NodeHandle &private_nh);

  bool load(RosNodeConfig &config, const std::string &default_config_path) const;

private:
  bool loadNavDefaults(RosNodeConfig &config) const;
  bool validateConfig(const RosNodeConfig &config) const;

  ros::NodeHandle private_nh_;
};
