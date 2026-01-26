#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <common_msgs/Float64Stamped.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>

namespace ros_behavior_tree
{

struct NavSample
{
  double value{0.0};
  ros::Time stamp;
  bool has_value{false};
};

struct BoolSample
{
  bool value{false};
  ros::Time stamp;
  ros::Time last_true_stamp;
  bool has_value{false};
};

struct NavData
{
  NavSample heading;
  NavSample speed;
  NavSample depth;
  NavSample yaw;
  NavSample pitch;
  NavSample roll;
  NavSample x;
  NavSample y;
  BoolSample deploy;
  BoolSample return_home;
  BoolSample speed_trigger;
};

class NavDataStore
{
public:
  void updateHeading(double value, const ros::Time &stamp, bool nav_style);
  void updateSpeed(double value, const ros::Time &stamp, bool nav_style);
  void updateDepth(double value, const ros::Time &stamp, bool nav_style);
  void updateYaw(double value, const ros::Time &stamp, bool nav_style);
  void updatePitch(double value, const ros::Time &stamp, bool nav_style);
  void updateRoll(double value, const ros::Time &stamp, bool nav_style);
  void updateX(double value, const ros::Time &stamp, bool nav_style);
  void updateY(double value, const ros::Time &stamp, bool nav_style);
  void updateDeploy(bool value, const ros::Time &stamp, bool nav_style);
  void updateReturn(bool value, const ros::Time &stamp, bool nav_style);
  void updateSpeedTrigger(bool value, const ros::Time &stamp, bool nav_style);
  void setTriggerHold(const ros::Duration &hold);
  void setDeployLatch(bool enabled);

  bool hasFreshData(const ros::Duration &timeout) const;
  bool preferredHeading(double &out, const ros::Duration &timeout) const;
  bool preferredSpeed(double &out, const ros::Duration &timeout) const;
  bool preferredDepth(double &out, const ros::Duration &timeout) const;
  bool preferredYaw(double &out, const ros::Duration &timeout) const;
  bool preferredPitch(double &out, const ros::Duration &timeout) const;
  bool preferredRoll(double &out, const ros::Duration &timeout) const;
  bool preferredX(double &out, const ros::Duration &timeout) const;
  bool preferredY(double &out, const ros::Duration &timeout) const;
  bool preferredDeploy(bool &out, const ros::Duration &timeout) const;
  bool preferredReturn(bool &out, const ros::Duration &timeout) const;
  bool preferredSpeedTrigger(bool &out, const ros::Duration &timeout) const;

private:
  NavData ros_data_;
  NavData nav_data_;

  static bool sampleFresh(const NavSample &sample, const ros::Duration &timeout);
  static bool sampleFresh(const BoolSample &sample, const ros::Duration &timeout);
  static bool getPreferredSample(const NavSample &nav_sample,
                                 const NavSample &ros_sample,
                                 const ros::Duration &timeout,
                                 double &out);
  bool getPreferredSample(const BoolSample &nav_sample,
                          const BoolSample &ros_sample,
                          const ros::Duration &timeout,
                          bool &out) const;
  bool sampleTriggered(const BoolSample &sample, const ros::Duration &timeout,
                       bool &out) const;

  ros::Duration trigger_hold_{0.0};
  bool latch_deploy_{false};
  bool deploy_latched_{false};
};

class HelmInterface;

class BehaviorTreeManager
{
public:
  BehaviorTreeManager(const ros::NodeHandle &nh, const NavDataStore *store);
  void tick();
  std::map<std::string, double> desiredValues() const;

private:
  ros::NodeHandle nh_;
  const NavDataStore *store_{nullptr};
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::PublisherZMQ> groot_publisher_;
  std::shared_ptr<class HelmInterface> helm_interface_;
  bool tree_loaded_{false};
};

}  // namespace ros_behavior_tree
