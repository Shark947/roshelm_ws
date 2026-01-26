#include <ros/ros.h>

#include <cmath>

#include <common_msgs/Float64Stamped.h>

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

class NavPublisher
{
public:
  explicit NavPublisher(ros::NodeHandle &nh)
      : nh_(nh)
  {
  }

  bool initialize(const std::string &vehicle_name)
  {
    if (vehicle_name.empty())
    {
      ROS_ERROR("[ros_behavior_tree] vehicle_name is empty for nav publisher");
      return false;
    }

    vehicle_name_ = vehicle_name;
    const std::string base = "/" + vehicle_name_ + "/current_";

    vx_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "vx", 10, &NavPublisher::vxCallback, this);
    vy_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "vy", 10, &NavPublisher::vyCallback, this);
    yaw_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "yaw", 10, &NavPublisher::yawCallback, this);
    pitch_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "pitch", 10, &NavPublisher::pitchCallback, this);
    roll_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "roll", 10, &NavPublisher::rollCallback, this);
    z_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "z", 10, &NavPublisher::zCallback, this);
    x_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "x", 10, &NavPublisher::xCallback, this);
    y_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
        base + "y", 10, &NavPublisher::yCallback, this);

    speed_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        base + "speed", 10);
    heading_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        base + "heading", 10);
    depth_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        base + "depth", 10);
    nav_speed_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_SPEED", 10);
    nav_heading_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_HEADING", 10);
    nav_depth_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_DEPTH", 10);
    nav_yaw_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_YAW", 10);
    nav_pitch_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_PITCH", 10);
    nav_roll_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_ROLL", 10);
    nav_x_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_X", 10);
    nav_y_pub_ = nh_.advertise<common_msgs::Float64Stamped>(
        "/" + vehicle_name_ + "/NAV_Y", 10);

    ROS_INFO_STREAM("[ros_behavior_tree] nav publisher started for vehicle: "
                    << vehicle_name_);
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
    nav_heading_pub_.publish(out);

    common_msgs::Float64Stamped nav_yaw = *msg;
    nav_yaw_pub_.publish(nav_yaw);
  }

  void pitchCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    common_msgs::Float64Stamped nav_pitch = *msg;
    nav_pitch_pub_.publish(nav_pitch);
  }

  void rollCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    common_msgs::Float64Stamped nav_roll = *msg;
    nav_roll_pub_.publish(nav_roll);
  }

  void zCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    common_msgs::Float64Stamped out;
    out.header.stamp = msg->header.stamp;
    out.data = -msg->data;
    depth_pub_.publish(out);
    nav_depth_pub_.publish(out);
  }

  void xCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    nav_x_pub_.publish(*msg);
  }

  void yCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    nav_y_pub_.publish(*msg);
  }

  void publishSpeed(const ros::Time &stamp)
  {
    if (!have_vx_ || !have_vy_)
      return;

    common_msgs::Float64Stamped out;
    out.header.stamp = stamp;
    if (!last_vx_stamp_.isZero() && !last_vy_stamp_.isZero())
    {
      out.header.stamp =
          last_vx_stamp_ > last_vy_stamp_ ? last_vx_stamp_ : last_vy_stamp_;
    }
    out.data = std::hypot(last_vx_, last_vy_);
    speed_pub_.publish(out);
    nav_speed_pub_.publish(out);
  }

  ros::NodeHandle nh_;
  std::string vehicle_name_;

  ros::Subscriber vx_sub_;
  ros::Subscriber vy_sub_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber roll_sub_;
  ros::Subscriber z_sub_;
  ros::Subscriber x_sub_;
  ros::Subscriber y_sub_;

  ros::Publisher speed_pub_;
  ros::Publisher heading_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher nav_speed_pub_;
  ros::Publisher nav_heading_pub_;
  ros::Publisher nav_depth_pub_;
  ros::Publisher nav_yaw_pub_;
  ros::Publisher nav_pitch_pub_;
  ros::Publisher nav_roll_pub_;
  ros::Publisher nav_x_pub_;
  ros::Publisher nav_y_pub_;

  double last_vx_{0.0};
  double last_vy_{0.0};
  ros::Time last_vx_stamp_;
  ros::Time last_vy_stamp_;
  bool have_vx_{false};
  bool have_vy_{false};
};
}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string vehicle_name;
  private_nh.param("vehicle_name", vehicle_name, std::string("auh"));

  NavPublisher publisher(nh);
  if (!publisher.initialize(vehicle_name))
    return 1;

  ros::spin();
  return 0;
}
