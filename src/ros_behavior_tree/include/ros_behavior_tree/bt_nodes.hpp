#pragma once

#include <ros/ros.h>
#include <common_msgs/Float64Stamped.h>

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

private:
  NavData ros_data_;
  NavData nav_data_;

  static bool sampleFresh(const NavSample &sample, const ros::Duration &timeout);
  static bool sampleFresh(const BoolSample &sample, const ros::Duration &timeout);
  static bool getPreferredSample(const NavSample &nav_sample,
                                 const NavSample &ros_sample,
                                 const ros::Duration &timeout,
                                 double &out);
  static bool getPreferredSample(const BoolSample &nav_sample,
                                 const BoolSample &ros_sample,
                                 const ros::Duration &timeout,
                                 bool &out);
};

class BehaviorTreeManager
{
public:
  void tick();
};

}  // namespace ros_behavior_tree
