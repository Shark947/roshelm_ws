#include <cmath>
#include <string>

#include <ros/ros.h>

#include <common_msgs/Float64Stamped.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizeHeadingDeg(double heading_deg)
{
  double normalized = std::fmod(heading_deg, 360.0);
  if (normalized < 0.0)
    normalized += 360.0;
  return normalized;
}

class RosNavPublish
{
public:
  explicit RosNavPublish(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
      : nh_(nh), private_nh_(private_nh)
  {
  }

  bool initialize()
  {
    if (!private_nh_.getParam("vehicle_name", vehicle_name_) || vehicle_name_.empty())
    {
      ROS_ERROR_STREAM("[RosNavPublish] vehicle_name param missing or empty!");
      return false;
    }

    const std::string base = "/" + vehicle_name_ + "/current_";
    vx_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "vx", 10,
                                                        &RosNavPublish::vxCallback, this);
    vy_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "vy", 10,
                                                        &RosNavPublish::vyCallback, this);
    yaw_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "yaw", 10,
                                                         &RosNavPublish::yawCallback, this);
    z_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(base + "z", 10,
                                                       &RosNavPublish::zCallback, this);

    speed_pub_ = nh_.advertise<common_msgs::Float64Stamped>(base + "speed", 10);
    heading_pub_ = nh_.advertise<common_msgs::Float64Stamped>(base + "heading", 10);
    depth_pub_ = nh_.advertise<common_msgs::Float64Stamped>(base + "depth", 10);

    ROS_INFO_STREAM("[RosNavPublish] started for vehicle: " << vehicle_name_);
    return true;
  }

private:
  void vxCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    last_vx_ = msg->data;
    last_vx_stamp_ = msg->header.stamp;
    have_vx_ = true;
    publishSpeed(msg->header.stamp);
  }

  void vyCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    last_vy_ = msg->data;
    last_vy_stamp_ = msg->header.stamp;
    have_vy_ = true;
    publishSpeed(msg->header.stamp);
  }

  void yawCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    const double heading_deg = normalizeHeadingDeg(msg->data * 180.0 / kPi);

    common_msgs::Float64Stamped out;
    out.header.stamp = msg->header.stamp;
    out.data = heading_deg;
    heading_pub_.publish(out);
  }

  void zCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    common_msgs::Float64Stamped out;
    out.header.stamp = msg->header.stamp;
    out.data = -msg->data;
    depth_pub_.publish(out);
  }

  void publishSpeed(const ros::Time &stamp)
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

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
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

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_nav_publish");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  RosNavPublish publisher(nh, private_nh);
  if (!publisher.initialize())
    return 1;

  ros::spin();
  return 0;
}
