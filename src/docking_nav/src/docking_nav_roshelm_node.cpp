#include <ros/ros.h>

#include "HelmIvP.h"
#include "RosBridge.h"
#include "RosConfigLoader.h"
#include "RosNavPublisher.h"

namespace
{
struct DockingRosConfig
{
  RosNodeConfig base;
  std::string docking_mode_topic{"/docking/mode"};
  std::string docking_stationing_topic{"/docking/stationing"};
  std::string docking_constheight_topic{"/docking/constheight"};
  std::string docking_dockdepth_update_topic{"/docking/dockdepth_update"};
  std::string docking_dockhdg_updates_topic{"/docking/dockhdg_updates"};
  std::string docking_docking_falling_topic{"/docking/docking_falling"};
  std::string docking_manual_override_topic{"/docking/manual_override"};
  std::string docking_failed_topic{"/docking/docking_failed"};
};

class DockingRosConfigLoader
{
public:
  explicit DockingRosConfigLoader(ros::NodeHandle &private_nh)
      : private_nh_(private_nh), base_loader_(private_nh)
  {
  }

  bool load(DockingRosConfig &config,
            const std::string &default_config_path) const
  {
    if (!base_loader_.load(config.base, default_config_path))
      return false;

    private_nh_.param("docking_mode_topic", config.docking_mode_topic,
                      config.docking_mode_topic);
    private_nh_.param("docking_stationing_topic", config.docking_stationing_topic,
                      config.docking_stationing_topic);
    private_nh_.param("docking_constheight_topic", config.docking_constheight_topic,
                      config.docking_constheight_topic);
    private_nh_.param("docking_dockdepth_update_topic",
                      config.docking_dockdepth_update_topic,
                      config.docking_dockdepth_update_topic);
    private_nh_.param("docking_dockhdg_updates_topic",
                      config.docking_dockhdg_updates_topic,
                      config.docking_dockhdg_updates_topic);
    private_nh_.param("docking_docking_falling_topic",
                      config.docking_docking_falling_topic,
                      config.docking_docking_falling_topic);
    private_nh_.param("docking_manual_override_topic",
                      config.docking_manual_override_topic,
                      config.docking_manual_override_topic);
    private_nh_.param("docking_failed_topic", config.docking_failed_topic,
                      config.docking_failed_topic);

    return true;
  }

private:
  ros::NodeHandle private_nh_;
  RosConfigLoader base_loader_;
};

class DockingRosBridge : public RosBridge
{
public:
  DockingRosBridge(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                   const DockingRosConfig &config)
      : RosBridge(nh, private_nh, config.base), docking_config_(config)
  {
  }

  bool initialize()
  {
    if (!RosBridge::initialize())
      return false;

    docking_mode_sub_ =
        subscribeString(docking_config_.docking_mode_topic, "MODE");
    docking_stationing_sub_ =
        subscribeBoolean(docking_config_.docking_stationing_topic, "STATIONING");
    docking_constheight_sub_ =
        subscribeBoolean(docking_config_.docking_constheight_topic, "CONSTHEIGHT");
    docking_dockdepth_update_sub_ =
        subscribeString(docking_config_.docking_dockdepth_update_topic,
                        "DOCKDEPTH_UPDATE");
    docking_dockhdg_updates_sub_ =
        subscribeString(docking_config_.docking_dockhdg_updates_topic,
                        "DOCKHDG_UPDATES");
    docking_docking_falling_sub_ =
        subscribeBoolean(docking_config_.docking_docking_falling_topic,
                         "DOCKING_FALLING");
    docking_manual_override_sub_ =
        subscribeBoolean(docking_config_.docking_manual_override_topic,
                         "MOOS_MANUAL_OVERIDE");
    docking_failed_sub_ =
        subscribeBoolean(docking_config_.docking_failed_topic, "DOCKINGFAILED");

    return true;
  }

private:
  DockingRosConfig docking_config_;
  ros::Subscriber docking_mode_sub_;
  ros::Subscriber docking_stationing_sub_;
  ros::Subscriber docking_constheight_sub_;
  ros::Subscriber docking_dockdepth_update_sub_;
  ros::Subscriber docking_dockhdg_updates_sub_;
  ros::Subscriber docking_docking_falling_sub_;
  ros::Subscriber docking_manual_override_sub_;
  ros::Subscriber docking_failed_sub_;
};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "docking_nav_roshelm_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  DockingRosConfig config;
  DockingRosConfigLoader loader(private_nh);
  if (!loader.load(config, DOCKING_NAV_DEFAULT_CONFIG_PATH))
  {
    ROS_ERROR("Failed to load docking ROS Helm configuration");
    return 1;
  }

  HelmIvP helm;
  helm.setAppName(config.base.node_name);
  helm.setConfigPath(config.base.config_path);
  helm.setRegisterVariablesPath(config.base.register_variables_path);

  if (!helm.OnStartUp())
  {
    ROS_ERROR("HelmIvP startup failed");
    return 1;
  }

  DockingRosBridge bridge(nh, private_nh, config);
  if (!bridge.initialize())
  {
    ROS_ERROR("DockingRosBridge initialization failed");
    return 1;
  }

  RosNavPublisher nav_publisher(nh);
  if (!nav_publisher.initialize(config.base.vehicle_name))
  {
    ROS_ERROR("RosNavPublisher initialization failed");
    return 1;
  }

  ros::Rate rate(config.base.loop_frequency);
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
