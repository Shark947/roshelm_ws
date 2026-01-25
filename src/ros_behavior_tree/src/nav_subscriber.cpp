#include "ros_behavior_tree/nav_subscriber.hpp"

namespace
{
ros::Time resolveStamp(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  if (msg->header.stamp.isZero())
    return ros::Time::now();
  return msg->header.stamp;
}
}  // namespace

namespace ros_behavior_tree
{

NavDataSubscriber::NavDataSubscriber(ros::NodeHandle &nh,
                                     const NavTopics &topics,
                                     NavDataStore &store)
{
  ros_heading_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_heading, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateHeading(msg->data, resolveStamp(msg), false);
      });
  ros_speed_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_speed, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateSpeed(msg->data, resolveStamp(msg), false);
      });
  ros_depth_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_depth, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateDepth(msg->data, resolveStamp(msg), false);
      });
  ros_yaw_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_yaw, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateYaw(msg->data, resolveStamp(msg), false);
      });
  ros_pitch_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_pitch, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updatePitch(msg->data, resolveStamp(msg), false);
      });
  ros_roll_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_roll, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateRoll(msg->data, resolveStamp(msg), false);
      });
  ros_x_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_x, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateX(msg->data, resolveStamp(msg), false);
      });
  ros_y_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.ros_y, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateY(msg->data, resolveStamp(msg), false);
      });
  ros_deploy_sub_ = nh.subscribe<std_msgs::Bool>(
      topics.ros_deploy, 10,
      [&store](const std_msgs::Bool::ConstPtr &msg) {
        store.updateDeploy(msg->data, ros::Time::now(), false);
      });
  ros_return_sub_ = nh.subscribe<std_msgs::Bool>(
      topics.ros_return, 10,
      [&store](const std_msgs::Bool::ConstPtr &msg) {
        store.updateReturn(msg->data, ros::Time::now(), false);
      });

  nav_heading_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_heading, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateHeading(msg->data, resolveStamp(msg), true);
      });
  nav_speed_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_speed, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateSpeed(msg->data, resolveStamp(msg), true);
      });
  nav_depth_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_depth, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateDepth(msg->data, resolveStamp(msg), true);
      });
  nav_yaw_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_yaw, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateYaw(msg->data, resolveStamp(msg), true);
      });
  nav_pitch_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_pitch, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updatePitch(msg->data, resolveStamp(msg), true);
      });
  nav_roll_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_roll, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateRoll(msg->data, resolveStamp(msg), true);
      });
  nav_x_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_x, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateX(msg->data, resolveStamp(msg), true);
      });
  nav_y_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
      topics.nav_y, 10,
      [&store](const common_msgs::Float64Stamped::ConstPtr &msg) {
        store.updateY(msg->data, resolveStamp(msg), true);
      });
  nav_deploy_sub_ = nh.subscribe<std_msgs::Bool>(
      topics.nav_deploy, 10,
      [&store](const std_msgs::Bool::ConstPtr &msg) {
        store.updateDeploy(msg->data, ros::Time::now(), true);
      });
  nav_return_sub_ = nh.subscribe<std_msgs::Bool>(
      topics.nav_return, 10,
      [&store](const std_msgs::Bool::ConstPtr &msg) {
        store.updateReturn(msg->data, ros::Time::now(), true);
      });
}

}  // namespace ros_behavior_tree
