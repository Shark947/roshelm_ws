#ifndef ROS_HELM_ROS_NAV_PUBLISHER_H
#define ROS_HELM_ROS_NAV_PUBLISHER_H

#include <string>

#include <ros/ros.h>

#include <common_msgs/Float64Stamped.h>

class RosNavPublisher
{
public:
  explicit RosNavPublisher(ros::NodeHandle &nh);

  bool initialize(const std::string &vehicle_name);

private:
  void vxCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void vyCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void yawCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void zCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void publishSpeed(const ros::Time &stamp);

  ros::NodeHandle nh_;
  std::string vehicle_name_;

  ros::Subscriber vx_sub_;
  ros::Subscriber vy_sub_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber z_sub_;

  ros::Publisher speed_pub_;
  ros::Publisher heading_pub_;
  ros::Publisher depth_pub_;

  bool have_vx_{false};
  bool have_vy_{false};
  double last_vx_{0.0};
  double last_vy_{0.0};
  ros::Time last_vx_stamp_;
  ros::Time last_vy_stamp_;
};

#endif  // ROS_HELM_ROS_NAV_PUBLISHER_H
