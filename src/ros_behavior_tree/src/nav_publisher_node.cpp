#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <mutex>

#include <common_msgs/Float64Stamped.h>
#include <tf/transform_datatypes.h>

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
    updateCurrentSpeed(msg, true);
    handleVelocity(msg, true);
  }

  void vyCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    updateCurrentSpeed(msg, false);
    handleVelocity(msg, false);
  }

  void yawCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    publishCurrentHeading(msg);
    handleAttitude(msg, AttitudeKey::kYaw);
  }

  void pitchCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    handleAttitude(msg, AttitudeKey::kPitch);
  }

  void rollCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    handleAttitude(msg, AttitudeKey::kRoll);
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

  struct SpeedCache
  {
    double vx{0.0};
    double vy{0.0};
    ros::Time stamp_vx;
    ros::Time stamp_vy;
    bool has_vx{false};
    bool has_vy{false};

    void reset()
    {
      vx = 0.0;
      vy = 0.0;
      stamp_vx = ros::Time();
      stamp_vy = ros::Time();
      has_vx = false;
      has_vy = false;
    }
  };

  struct AttitudeCache
  {
    double yaw{0.0};
    double pitch{0.0};
    double roll{0.0};
    ros::Time stamp_yaw;
    ros::Time stamp_pitch;
    ros::Time stamp_roll;
    bool has_yaw{false};
    bool has_pitch{false};
    bool has_roll{false};

    void reset()
    {
      yaw = 0.0;
      pitch = 0.0;
      roll = 0.0;
      stamp_yaw = ros::Time();
      stamp_pitch = ros::Time();
      stamp_roll = ros::Time();
      has_yaw = false;
      has_pitch = false;
      has_roll = false;
    }
  };

  enum class AttitudeKey
  {
    kYaw,
    kPitch,
    kRoll
  };

  void updateCurrentSpeed(const common_msgs::Float64Stamped::ConstPtr &msg,
                          bool is_vx)
  {
    if (is_vx)
    {
      have_vx_ = true;
      last_vx_ = msg->data;
      last_vx_stamp_ = msg->header.stamp;
    }
    else
    {
      have_vy_ = true;
      last_vy_ = msg->data;
      last_vy_stamp_ = msg->header.stamp;
    }

    if (!have_vx_ || !have_vy_)
      return;

    common_msgs::Float64Stamped out;
    out.header.stamp = msg->header.stamp;
    if (!last_vx_stamp_.isZero() && !last_vy_stamp_.isZero())
    {
      out.header.stamp = last_vx_stamp_ > last_vy_stamp_
                             ? last_vx_stamp_
                             : last_vy_stamp_;
    }
    out.data = std::hypot(last_vx_, last_vy_);
    speed_pub_.publish(out);
  }

  void publishCurrentHeading(
      const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    const double heading_deg = normalizeHeadingDeg(msg->data * 180.0 / kPi);
    common_msgs::Float64Stamped out;
    out.header.stamp = msg->header.stamp;
    out.data = heading_deg;
    heading_pub_.publish(out);
  }

  void handleVelocity(const common_msgs::Float64Stamped::ConstPtr &msg,
                      bool is_vx)
  {
    constexpr double kStampTolSec = 0.02;
    bool ready = false;
    double vx = 0.0;
    double vy = 0.0;
    ros::Time stamp;

    {
      std::lock_guard<std::mutex> guard(speed_mutex_);
      if (is_vx)
      {
        speed_cache_.vx = msg->data;
        speed_cache_.stamp_vx = msg->header.stamp;
        speed_cache_.has_vx = true;
      }
      else
      {
        speed_cache_.vy = msg->data;
        speed_cache_.stamp_vy = msg->header.stamp;
        speed_cache_.has_vy = true;
      }

      if (speed_cache_.has_vx && speed_cache_.has_vy)
      {
        const double dt = std::fabs(
            (speed_cache_.stamp_vx - speed_cache_.stamp_vy).toSec());
        if (dt <= kStampTolSec)
        {
          vx = speed_cache_.vx;
          vy = speed_cache_.vy;
          stamp = speed_cache_.stamp_vx;
          if (speed_cache_.stamp_vy > stamp)
            stamp = speed_cache_.stamp_vy;
          ready = true;
          speed_cache_.reset();
        }
        else
        {
          const double value = msg->data;
          const ros::Time value_stamp = msg->header.stamp;
          speed_cache_.reset();
          if (is_vx)
          {
            speed_cache_.vx = value;
            speed_cache_.stamp_vx = value_stamp;
            speed_cache_.has_vx = true;
          }
          else
          {
            speed_cache_.vy = value;
            speed_cache_.stamp_vy = value_stamp;
            speed_cache_.has_vy = true;
          }
        }
      }
    }

    if (!ready)
      return;

    common_msgs::Float64Stamped out;
    out.header.stamp = stamp;
    out.data = std::hypot(vx, vy);
    nav_speed_pub_.publish(out);
  }

  void handleAttitude(const common_msgs::Float64Stamped::ConstPtr &msg,
                      AttitudeKey key)
  {
    constexpr double kStampTolSec = 0.02;
    bool ready = false;
    double yaw_r = 0.0;
    double pitch_r = 0.0;
    double roll_r = 0.0;
    ros::Time stamp;

    {
      std::lock_guard<std::mutex> guard(attitude_mutex_);
      if (key == AttitudeKey::kYaw)
      {
        attitude_cache_.yaw = msg->data;
        attitude_cache_.stamp_yaw = msg->header.stamp;
        attitude_cache_.has_yaw = true;
      }
      else if (key == AttitudeKey::kPitch)
      {
        attitude_cache_.pitch = msg->data;
        attitude_cache_.stamp_pitch = msg->header.stamp;
        attitude_cache_.has_pitch = true;
      }
      else
      {
        attitude_cache_.roll = msg->data;
        attitude_cache_.stamp_roll = msg->header.stamp;
        attitude_cache_.has_roll = true;
      }

      if (attitude_cache_.has_yaw && attitude_cache_.has_pitch &&
          attitude_cache_.has_roll)
      {
        const double dt_yp = std::fabs(
            (attitude_cache_.stamp_yaw - attitude_cache_.stamp_pitch).toSec());
        const double dt_yr = std::fabs(
            (attitude_cache_.stamp_yaw - attitude_cache_.stamp_roll).toSec());
        const double dt_pr = std::fabs(
            (attitude_cache_.stamp_pitch - attitude_cache_.stamp_roll).toSec());

        if (dt_yp <= kStampTolSec && dt_yr <= kStampTolSec &&
            dt_pr <= kStampTolSec)
        {
          yaw_r = attitude_cache_.yaw;
          pitch_r = attitude_cache_.pitch;
          roll_r = attitude_cache_.roll;

          stamp = attitude_cache_.stamp_yaw;
          if (attitude_cache_.stamp_pitch > stamp)
            stamp = attitude_cache_.stamp_pitch;
          if (attitude_cache_.stamp_roll > stamp)
            stamp = attitude_cache_.stamp_roll;

          ready = true;
          attitude_cache_.reset();
        }
        else
        {
          const double value = msg->data;
          const ros::Time value_stamp = msg->header.stamp;
          attitude_cache_.reset();
          if (key == AttitudeKey::kYaw)
          {
            attitude_cache_.yaw = value;
            attitude_cache_.stamp_yaw = value_stamp;
            attitude_cache_.has_yaw = true;
          }
          else if (key == AttitudeKey::kPitch)
          {
            attitude_cache_.pitch = value;
            attitude_cache_.stamp_pitch = value_stamp;
            attitude_cache_.has_pitch = true;
          }
          else
          {
            attitude_cache_.roll = value;
            attitude_cache_.stamp_roll = value_stamp;
            attitude_cache_.has_roll = true;
          }
        }
      }
    }

    if (!ready)
      return;

    tf::Quaternion q;
    q.setRPY(roll_r, pitch_r, yaw_r);
    const tf::Matrix3x3 R(q);

    const tf::Matrix3x3 A(0, 1, 0,
                          -1, 0, 0,
                          0, 0, 1);

    const tf::Matrix3x3 M = R * A;

    auto clamp = [](double x, double lo, double hi) {
      return std::max(lo, std::min(hi, x));
    };

    const double s = clamp(M[2][1], -1.0, 1.0);
    const double nav_pitch = std::asin(s);

    const double c = std::cos(nav_pitch);
    double nav_yaw = 0.0;
    double nav_roll = 0.0;

    constexpr double eps = 1e-9;
    if (std::fabs(c) > eps)
    {
      nav_yaw = std::atan2(-M[0][1], M[1][1]);
      nav_roll = std::atan2(-M[2][0], M[2][2]);
    }
    else
    {
      nav_roll = 0.0;
      nav_yaw = std::atan2(M[1][0], M[0][0]);
    }

    common_msgs::Float64Stamped nav_yaw_msg;
    nav_yaw_msg.header.stamp = stamp;
    nav_yaw_msg.data = nav_yaw;
    nav_yaw_pub_.publish(nav_yaw_msg);

    common_msgs::Float64Stamped nav_pitch_msg;
    nav_pitch_msg.header.stamp = stamp;
    nav_pitch_msg.data = nav_pitch;
    nav_pitch_pub_.publish(nav_pitch_msg);

    common_msgs::Float64Stamped nav_roll_msg;
    nav_roll_msg.header.stamp = stamp;
    nav_roll_msg.data = nav_roll;
    nav_roll_pub_.publish(nav_roll_msg);

    constexpr double kRadToDeg = 180.0 / M_PI;
    const double nav_heading =
        normalizeHeadingDeg(-nav_yaw * kRadToDeg);

    common_msgs::Float64Stamped heading_msg;
    heading_msg.header.stamp = stamp;
    heading_msg.data = nav_heading;
    nav_heading_pub_.publish(heading_msg);
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

  std::mutex speed_mutex_;
  SpeedCache speed_cache_;
  std::mutex attitude_mutex_;
  AttitudeCache attitude_cache_;

  bool have_vx_{false};
  bool have_vy_{false};
  double last_vx_{0.0};
  double last_vy_{0.0};
  ros::Time last_vx_stamp_;
  ros::Time last_vy_stamp_;
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
