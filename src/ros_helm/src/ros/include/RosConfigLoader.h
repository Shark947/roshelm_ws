#pragma once

#include <map>
#include <string>
#include <ros/ros.h>

struct RosNodeConfig
{
  std::string node_name{"ros_bridge"};
  std::string config_path;
  std::string register_variables_path;
  std::string vehicle_name;
  std::string current_heading_topic;
  std::string current_speed_topic;
  std::string current_depth_topic;
  std::string current_x_topic;
  std::string current_y_topic;
  std::string current_z_topic;
  std::string current_vx_topic;
  std::string current_vy_topic;
  std::string current_yaw_topic;
  std::string current_pitch_topic;
  std::string current_roll_topic;
  std::string desired_heading_topic;
  std::string desired_speed_topic;
  std::string desired_depth_topic;
  std::string deploy_topic;
  std::string return_topic;
  std::string frame_id{"map"};
  double loop_frequency{10.0};
  double status_log_period{2.0};
  bool deploy_default{true};
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
