#include "RosConfigLoader.h"

#include <set>
#include <XmlRpcValue.h>

namespace
{
const char *kNavDefaultsNamespace = "nav_defaults";
}

RosConfigLoader::RosConfigLoader(ros::NodeHandle &private_nh)
    : private_nh_(private_nh)
{
}

bool RosConfigLoader::load(RosNodeConfig &config,
                           const std::string &default_config_path) const
{
  private_nh_.param("node_name", config.node_name, config.node_name);
  private_nh_.param("config_path", config.config_path, default_config_path);
  private_nh_.param("register_variables_path", config.register_variables_path,
                    std::string("src/helm/config/registerVariables.yaml"));
  private_nh_.param("vehicle_name", config.vehicle_name, config.vehicle_name);

  private_nh_.param("loop_frequency", config.loop_frequency,
                    config.loop_frequency);
  private_nh_.param("frame_id", config.frame_id, config.frame_id);

  private_nh_.param("current_heading_topic", config.current_heading_topic,
                    config.current_heading_topic);
  private_nh_.param("current_speed_topic", config.current_speed_topic,
                    config.current_speed_topic);
  private_nh_.param("current_depth_topic", config.current_depth_topic,
                    config.current_depth_topic);

  private_nh_.param("desired_heading_topic", config.desired_heading_topic,
                    config.desired_heading_topic);
  private_nh_.param("desired_speed_topic", config.desired_speed_topic,
                    config.desired_speed_topic);
  private_nh_.param("desired_depth_topic", config.desired_depth_topic,
                    config.desired_depth_topic);

  if (config.current_heading_topic.empty())
    config.current_heading_topic = "/" + config.vehicle_name + "/current_heading";
  if (config.current_speed_topic.empty())
    config.current_speed_topic = "/" + config.vehicle_name + "/current_speed";
  if (config.current_depth_topic.empty())
    config.current_depth_topic = "/" + config.vehicle_name + "/current_depth";

  if (config.desired_heading_topic.empty())
    config.desired_heading_topic = "/" + config.vehicle_name + "/desired_heading";
  if (config.desired_speed_topic.empty())
    config.desired_speed_topic = "/" + config.vehicle_name + "/desired_speed";
  if (config.desired_depth_topic.empty())
    config.desired_depth_topic = "/" + config.vehicle_name + "/desired_depth";

  loadNavDefaults(config);

  if (!validateConfig(config))
    return false;

  ROS_INFO_STREAM("[ros_bridge] config_path=" << config.config_path
                  << " loop_frequency=" << config.loop_frequency);
  return true;
}

bool RosConfigLoader::loadNavDefaults(RosNodeConfig &config) const
{
  std::map<std::string, double> defaults = {
      {"NAV_X", 0.0},       {"NAV_Y", 0.0},
      {"NAV_HEADING", 0.0}, {"NAV_SPEED", 0.0},
      {"NAV_DEPTH", 0.0},
  };

  XmlRpc::XmlRpcValue yaml_defaults;
  if (private_nh_.getParam(kNavDefaultsNamespace, yaml_defaults) &&
      yaml_defaults.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (const auto &entry : defaults)
    {
      const std::string key = entry.first;
      if (yaml_defaults.hasMember(key))
      {
        try
        {
          double value = static_cast<double>(yaml_defaults[key]);
          defaults[key] = value;
        }
        catch (const XmlRpc::XmlRpcException &ex)
        {
          ROS_ERROR_STREAM("Invalid default for " << key << ": " << ex.getMessage());
        }
      }
    }
  }

  config.nav_defaults = defaults;
  return true;
}

bool RosConfigLoader::validateConfig(const RosNodeConfig &config) const
{
  if (config.config_path.empty())
  {
    ROS_ERROR("ros_bridge: config_path is empty");
    return false;
  }

  if (config.loop_frequency <= 0.0)
  {
    ROS_ERROR("ros_bridge: loop_frequency must be positive");
    return false;
  }

  const std::set<std::string> required_topics = {
      config.current_heading_topic, config.current_speed_topic,
      config.current_depth_topic,  config.desired_heading_topic,
      config.desired_speed_topic,  config.desired_depth_topic};
  for (const auto &topic : required_topics)
  {
    if (topic.empty())
    {
      ROS_ERROR("ros_bridge: required topic remapping is empty");
      return false;
    }
  }

  return true;
}
