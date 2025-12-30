#include "RosNavPublisher.h"

#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeHeadingDeg(double heading_deg)
{
  double normalized = std::fmod(heading_deg, 360.0);
  if (normalized < 0.0)
    normalized += 360.0;
  return normalized;
}
}  // namespace

RosNavPublisher::RosNavPublisher(ros::NodeHandle &nh)
    : nh_(nh)
{
}

bool RosNavPublisher::initialize(const std::string &vehicle_name)
{
  if (vehicle_name.empty())
  {
    ROS_ERROR("[RosNavPublisher] vehicle_name is empty");
    return false;
  }

  vehicle_name_ = vehicle_name;

  const std::string base = "/" + vehicle_name_ + "/current_";
  vx_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "vx", 10,
                                                      &RosNavPublisher::vxCallback, this);
  vy_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "vy", 10,
                                                      &RosNavPublisher::vyCallback, this);
  yaw_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "yaw", 10,
                                                       &RosNavPublisher::yawCallback, this);
  z_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "z", 10,
                                                     &RosNavPublisher::zCallback, this);

  speed_pub_ = nh_.advertise<common_msgs::Float64Stamped>(base + "speed", 10);
  heading_pub_ = nh_.advertise<common_msgs::Float64Stamped>(base + "heading", 10);
  depth_pub_ = nh_.advertise<common_msgs::Float64Stamped>(base + "depth", 10);

  ROS_INFO_STREAM("[RosNavPublisher] started for vehicle: " << vehicle_name_);
  return true;
}

void RosNavPublisher::vxCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  last_vx_ = msg->data;
  last_vx_stamp_ = msg->header.stamp;
  have_vx_ = true;
  publishSpeed(msg->header.stamp);
}

void RosNavPublisher::vyCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  last_vy_ = msg->data;
  last_vy_stamp_ = msg->header.stamp;
  have_vy_ = true;
  publishSpeed(msg->header.stamp);
}

void RosNavPublisher::yawCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  const double heading_deg = normalizeHeadingDeg(msg->data * 180.0 / kPi);

  common_msgs::Float64Stamped out;
  out.header.stamp = msg->header.stamp;
  out.data = heading_deg;
  heading_pub_.publish(out);
}

void RosNavPublisher::zCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  common_msgs::Float64Stamped out;
  out.header.stamp = msg->header.stamp;
  out.data = -msg->data;
  depth_pub_.publish(out);
}

void RosNavPublisher::publishSpeed(const ros::Time &stamp)
{
  if (!have_vx_ || !have_vy_)
    return;

  common_msgs::Float64Stamped out;
  out.header.stamp = stamp;
  if (!last_vx_stamp_.isZero() && !last_vy_stamp_.isZero())
  {
    out.header.stamp = last_vx_stamp_ > last_vy_stamp_ ? last_vx_stamp_ : last_vy_stamp_;
  }
  out.data = std::hypot(last_vx_, last_vy_);
  speed_pub_.publish(out);
}
