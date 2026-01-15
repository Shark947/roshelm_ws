#include "RosConfigLoader.h"

#include <set>
#include <sys/stat.h>
#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include <ros/package.h>

namespace
{
const char *kNavDefaultsNamespace = "nav_defaults";

bool fileExists(const std::string &path)
{
  struct stat sb;
  return stat(path.c_str(), &sb) == 0;
}

bool isAbsolutePath(const std::string &path)
{
  return !path.empty() && path.front() == '/';
}

std::string resolvePath(const std::string &path,
                        const std::string &package_path)
{
  if (path.empty())
    return path;

  if (isAbsolutePath(path))
    return path;

  if (fileExists(path))
    return path;

  if (!package_path.empty())
  {
    const std::string candidate = package_path + "/" + path;
    if (fileExists(candidate))
      return candidate;
  }

  return path;
}
}

RosConfigLoader::RosConfigLoader(ros::NodeHandle &private_nh)
    : private_nh_(private_nh)
{
}

namespace
{
bool loadCommandTopics(const ros::NodeHandle &private_nh,
                       const std::string &param_name,
                       std::map<std::string, std::string> &topics)
{
  XmlRpc::XmlRpcValue raw_value;
  if (!private_nh.getParam(param_name, raw_value))
  {
    return true;
  }

  if (raw_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM("ros_bridge: " << param_name
                                    << " must be a map of key: topic");
    return false;
  }

  topics.clear();
  for (auto it = raw_value.begin(); it != raw_value.end(); ++it)
  {
    if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("ros_bridge: " << param_name << "[" << it->first
                                      << "] must be a string");
      return false;
    }
    topics[it->first] = static_cast<std::string>(it->second);
  }
  return true;
}
}  // namespace

bool RosConfigLoader::load(RosNodeConfig &config,
                           const std::string &default_config_path) const
{
  const std::string package_path = ros::package::getPath("ros_helm");

  private_nh_.param("node_name", config.node_name, config.node_name);
  private_nh_.param("config_path", config.config_path, default_config_path);

  config.config_path = resolvePath(config.config_path, package_path);

  if (!fileExists(config.config_path))
  {
    ROS_ERROR_STREAM("ros_bridge: config_path does not exist: "
                     << config.config_path);
    return false;
  }
  private_nh_.param("vehicle_name", config.vehicle_name, config.vehicle_name);
  private_nh_.param("deploy_topic", config.deploy_topic, config.deploy_topic);
  private_nh_.param("return_topic", config.return_topic, config.return_topic);
  private_nh_.param("deploy_default", config.deploy_default,
                    config.deploy_default);
  private_nh_.param("return_default", config.return_default,
                    config.return_default);
  private_nh_.param("status_log_period", config.status_log_period,
                    config.status_log_period);
  if (!loadCommandTopics(private_nh_, "bool_command_topics",
                         config.bool_command_topics))
    return false;
  if (!loadCommandTopics(private_nh_, "string_command_topics",
                         config.string_command_topics))
    return false;
  if (!loadCommandTopics(private_nh_, "mode_state_topics",
                         config.mode_state_topics))
    return false;

  private_nh_.param("loop_frequency", config.loop_frequency,
                    config.loop_frequency);
  private_nh_.param("frame_id", config.frame_id, config.frame_id);

  private_nh_.param("current_heading_topic", config.current_heading_topic,
                    config.current_heading_topic);
  private_nh_.param("current_speed_topic", config.current_speed_topic,
                    config.current_speed_topic);
  private_nh_.param("current_depth_topic", config.current_depth_topic,
                    config.current_depth_topic);
  private_nh_.param("current_x_topic", config.current_x_topic,
                    config.current_x_topic);
  private_nh_.param("current_y_topic", config.current_y_topic,
                    config.current_y_topic);
  private_nh_.param("current_z_topic", config.current_z_topic,
                    config.current_z_topic);
  private_nh_.param("current_vx_topic", config.current_vx_topic,
                    config.current_vx_topic);
  private_nh_.param("current_vy_topic", config.current_vy_topic,
                    config.current_vy_topic);
  private_nh_.param("current_yaw_topic", config.current_yaw_topic,
                    config.current_yaw_topic);
  private_nh_.param("current_pitch_topic", config.current_pitch_topic,
                    config.current_pitch_topic);
  private_nh_.param("current_roll_topic", config.current_roll_topic,
                    config.current_roll_topic);
  private_nh_.param("nav_x_topic", config.nav_x_topic, config.nav_x_topic);
  private_nh_.param("nav_y_topic", config.nav_y_topic, config.nav_y_topic);
  private_nh_.param("nav_depth_topic", config.nav_depth_topic,
                    config.nav_depth_topic);
  private_nh_.param("nav_speed_topic", config.nav_speed_topic,
                    config.nav_speed_topic);
  private_nh_.param("nav_heading_topic", config.nav_heading_topic,
                    config.nav_heading_topic);
  private_nh_.param("nav_yaw_topic", config.nav_yaw_topic, config.nav_yaw_topic);
  private_nh_.param("nav_pitch_topic", config.nav_pitch_topic,
                    config.nav_pitch_topic);
  private_nh_.param("nav_roll_topic", config.nav_roll_topic,
                    config.nav_roll_topic);

  private_nh_.param("desired_heading_topic", config.desired_heading_topic,
                    config.desired_heading_topic);
  private_nh_.param("desired_speed_topic", config.desired_speed_topic,
                    config.desired_speed_topic);
  private_nh_.param("desired_depth_topic", config.desired_depth_topic,
                    config.desired_depth_topic);

  if (config.vehicle_name.empty())
  {
    ROS_ERROR("ros_bridge: vehicle_name is empty; ensure params.yaml is loaded in the node namespace");
    return false;
  }

  if (config.current_heading_topic.empty())
    config.current_heading_topic = "/" + config.vehicle_name + "/current_heading";
  if (config.current_speed_topic.empty())
    config.current_speed_topic = "/" + config.vehicle_name + "/current_speed";
  if (config.current_depth_topic.empty())
    config.current_depth_topic = "/" + config.vehicle_name + "/current_depth";
  if (config.current_x_topic.empty())
    config.current_x_topic = "/" + config.vehicle_name + "/current_x";
  if (config.current_y_topic.empty())
    config.current_y_topic = "/" + config.vehicle_name + "/current_y";
  if (config.current_z_topic.empty())
    config.current_z_topic = "/" + config.vehicle_name + "/current_z";
  if (config.current_vx_topic.empty())
    config.current_vx_topic = "/" + config.vehicle_name + "/current_vx";
  if (config.current_vy_topic.empty())
    config.current_vy_topic = "/" + config.vehicle_name + "/current_vy";
  if (config.current_yaw_topic.empty())
    config.current_yaw_topic = "/" + config.vehicle_name + "/current_yaw";
  if (config.current_pitch_topic.empty())
    config.current_pitch_topic = "/" + config.vehicle_name + "/current_pitch";
  if (config.current_roll_topic.empty())
    config.current_roll_topic = "/" + config.vehicle_name + "/current_roll";

  if (config.nav_x_topic.empty())
    config.nav_x_topic = "/" + config.vehicle_name + "/NAV_X";
  if (config.nav_y_topic.empty())
    config.nav_y_topic = "/" + config.vehicle_name + "/NAV_Y";
  if (config.nav_depth_topic.empty())
    config.nav_depth_topic = "/" + config.vehicle_name + "/NAV_DEPTH";
  if (config.nav_speed_topic.empty())
    config.nav_speed_topic = "/" + config.vehicle_name + "/NAV_SPEED";
  if (config.nav_heading_topic.empty())
    config.nav_heading_topic = "/" + config.vehicle_name + "/NAV_HEADING";
  if (config.nav_yaw_topic.empty())
    config.nav_yaw_topic = "/" + config.vehicle_name + "/NAV_YAW";
  if (config.nav_pitch_topic.empty())
    config.nav_pitch_topic = "/" + config.vehicle_name + "/NAV_PITCH";
  if (config.nav_roll_topic.empty())
    config.nav_roll_topic = "/" + config.vehicle_name + "/NAV_ROLL";

  if (config.desired_heading_topic.empty())
    config.desired_heading_topic = "/" + config.vehicle_name + "/desired_heading";
  if (config.desired_speed_topic.empty())
    config.desired_speed_topic = "/" + config.vehicle_name + "/desired_speed";
  if (config.desired_depth_topic.empty())
    config.desired_depth_topic = "/" + config.vehicle_name + "/desired_depth";

  if (config.deploy_topic.empty())
    config.deploy_topic = "/" + config.vehicle_name + "/DEPLOY";
  if (config.return_topic.empty())
    config.return_topic = "/" + config.vehicle_name + "/RETURN";

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

  if (config.status_log_period < 0.0)
  {
    ROS_ERROR("ros_bridge: status_log_period must be non-negative");
    return false;
  }

  if (config.vehicle_name.empty())
  {
    ROS_ERROR("ros_bridge: vehicle_name must be provided");
    return false;
  }

  const std::set<std::string> required_topics = {
      config.current_heading_topic, config.current_speed_topic,
      config.current_depth_topic,  config.current_x_topic,
      config.current_y_topic,      config.current_z_topic,
      config.current_vx_topic,     config.current_vy_topic,
      config.current_yaw_topic,    config.current_pitch_topic,
      config.current_roll_topic,   config.nav_x_topic,
      config.nav_y_topic,          config.nav_depth_topic,
      config.nav_speed_topic,      config.nav_heading_topic,
      config.nav_yaw_topic,        config.nav_pitch_topic,
      config.nav_roll_topic,       config.desired_heading_topic,
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
