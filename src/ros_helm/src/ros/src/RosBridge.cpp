#include "RosBridge.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <ctime>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <list>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include "MBUtils.h"

RosBridge::RosBridge(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                     const RosNodeConfig &config)
    : nh_(nh), private_nh_(private_nh), config_(config)
{
}

bool RosBridge::initialize()
{
  if (!setupLogDirectory())
    return false;

  x_sub_ = subscribeCurrent(config_.current_x_topic, "NAV_X");
  y_sub_ = subscribeCurrent(config_.current_y_topic, "NAV_Y");
  z_sub_ = subscribeCurrent(config_.current_z_topic, "NAV_Z");
  vx_sub_ = subscribeCurrent(config_.current_vx_topic, "NAV_VX");
  vy_sub_ = subscribeCurrent(config_.current_vy_topic, "NAV_VY");
  yaw_sub_ = subscribeCurrent(config_.current_yaw_topic, "ROS_YAW");
  pitch_sub_ = subscribeCurrent(config_.current_pitch_topic, "ROS_PITCH");
  roll_sub_ = subscribeCurrent(config_.current_roll_topic, "ROS_ROLL");

  command_publisher_ = std::make_unique<RosCommandPublisher>(
      nh_, config_,
      [this](const std::string &key, bool value, const ros::Time &stamp) {
        enqueueBoolValue(key, value, stamp);
      },
      [this]() { return getNavStamp(); });
  if (!command_publisher_->initialize())
    return false;

  desired_scalar_pubs_["DESIRED_HEADING"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_heading_topic, 10);
  desired_scalar_pubs_["DESIRED_SPEED"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_speed_topic, 10);
  desired_scalar_pubs_["DESIRED_DEPTH"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_depth_topic, 10);

  const ros::Time stamp = ros::Time::now();
  for (const auto &entry : config_.nav_defaults)
    enqueueNavValue(entry.first, entry.second, stamp);

  return true;
}

void RosBridge::enqueueNavValue(const std::string &key, double value,
                                const ros::Time &stamp)
{
  std::lock_guard<std::mutex> guard(mail_mutex_);
  pending_mail_.emplace_back(static_cast<char>(MsgType::Notify), key, value,
                             stamp.toSec(), "ros_bridge");

  logValue(key, value, stamp);
}

void RosBridge::enqueueBoolValue(const std::string &key, bool value,
                                 const ros::Time &stamp)
{
  const std::string text = value ? "true" : "false";

  std::lock_guard<std::mutex> guard(mail_mutex_);
  pending_mail_.emplace_back(static_cast<char>(MsgType::Notify), key, text,
                             stamp.toSec(), "ros_bridge");
}

ros::Subscriber RosBridge::subscribeCurrent(const std::string &topic,
                                            const std::string &nav_key)
{
  if (topic.empty())
    return {};

  return nh_.subscribe<common_msgs::Float64Stamped>(
      topic, 10,
      [this, nav_key](const common_msgs::Float64Stamped::ConstPtr &msg) {
        this->currentValueCallback(msg, nav_key);
      });
}

void RosBridge::currentValueCallback(
    const common_msgs::Float64Stamped::ConstPtr &msg,
    const std::string &nav_key)
{
  {
    std::lock_guard<std::mutex> guard(nav_stamp_mutex_);
    last_nav_stamp_ = msg->header.stamp;
  }

  if (nav_key == "NAV_Z")
  {
    enqueueNavValue("NAV_DEPTH", -msg->data, msg->header.stamp);
    enqueueNavValue(nav_key, msg->data, msg->header.stamp);
    return;
  }
  if (nav_key == "NAV_VX" || nav_key == "NAV_VY")
  {
    constexpr double kStampTolSec = 0.02;
    bool ready = false;
    double vx = 0.0;
    double vy = 0.0;
    ros::Time stamp;

    {
      std::lock_guard<std::mutex> guard(speed_mutex_);
      if (nav_key == "NAV_VX")
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
          if (nav_key == "NAV_VX")
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

    if (ready)
    {
      const double nav_speed = std::hypot(vx, vy);
      enqueueNavValue("NAV_SPEED", nav_speed, stamp);
    }

    enqueueNavValue(nav_key, msg->data, msg->header.stamp);
    return;
  }
  if (nav_key == "ROS_YAW" || nav_key == "ROS_PITCH" || nav_key == "ROS_ROLL")
  {
    constexpr double kStampTolSec = 0.02;
    bool ready = false;
    double yaw_r = 0.0;
    double pitch_r = 0.0;
    double roll_r = 0.0;
    ros::Time stamp;

    {
      std::lock_guard<std::mutex> guard(attitude_mutex_);

      if (nav_key == "ROS_YAW")
      {
        attitude_cache_.yaw = msg->data;
        attitude_cache_.stamp_yaw = msg->header.stamp;
        attitude_cache_.has_yaw = true;
      }
      else if (nav_key == "ROS_PITCH")
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
          if (nav_key == "ROS_YAW")
          {
            attitude_cache_.yaw = value;
            attitude_cache_.stamp_yaw = value_stamp;
            attitude_cache_.has_yaw = true;
          }
          else if (nav_key == "ROS_PITCH")
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

    enqueueNavValue("NAV_YAW", nav_yaw, stamp);
    enqueueNavValue("NAV_PITCH", nav_pitch, stamp);
    enqueueNavValue("NAV_ROLL", nav_roll, stamp);
    constexpr double kRadToDeg = 180.0 / M_PI;
    const double nav_heading = std::fmod(-nav_yaw * kRadToDeg + 360.0, 360.0);
    enqueueNavValue("NAV_HEADING", nav_heading, stamp);
    return;
  }
  enqueueNavValue(nav_key, msg->data, msg->header.stamp);
}

ros::Time RosBridge::getNavStamp() const
{
  std::lock_guard<std::mutex> guard(nav_stamp_mutex_);
  return last_nav_stamp_;
}

void RosBridge::deliverPending(HelmIvP &helm)
{
  std::list<HelmMsg> mail;
  {
    std::lock_guard<std::mutex> guard(mail_mutex_);
    mail.swap(pending_mail_);
  }

  if (mail.empty())
    return;

  helm.OnNewMail(mail);
}

void RosBridge::publishDesired(const HelmIvP &helm)
{
  const auto desired = collectDesiredDoubles(helm);
  if (desired.empty())
    return;

  auto publish_scalar = [&](const std::string &key, ros::Publisher &pub) {
    auto it = desired.find(key);
    if (it != desired.end() && pub)
    {
      std_msgs::Float64 msg;
      if (key == "DESIRED_HEADING")
      {
        msg.data = std::fmod(90.0 - it->second + 360.0, 360.0);
      }
      else
      {
        msg.data = it->second;
      }
      pub.publish(msg);
      logValue(key, msg.data, ros::Time::now());
    }
  };

  for (auto &[key, pub] : desired_scalar_pubs_)
    publish_scalar(key, pub);
}

std::map<std::string, double> RosBridge::collectDesiredDoubles(
    const HelmIvP &helm) const
{
  std::map<std::string, double> desired;

  for (const auto &entry : helm.getOutgoingDoubles())
  {
    if (entry.first.rfind("DESIRED_", 0) == 0)
      desired[entry.first] = entry.second;
  }

  const auto &domain = helm.getIvPDomain();
  const auto &report = helm.getHelmReport();
  for (int i = 0; i < domain.size(); ++i)
  {
    std::string domain_var = domain.getVarName(i);
    std::string desired_key = "DESIRED_" + toupper(domain_var);
    if (desired_key == "DESIRED_COURSE")
      desired_key = "DESIRED_HEADING";

    if (report.hasDecision(domain_var))
      desired[desired_key] = report.getDecision(domain_var);
  }

  return desired;
}

bool RosBridge::setupLogDirectory()
{
  static const std::array<std::string, 11> kLogKeys = {
      "NAV_X", "NAV_Y", "NAV_HEADING", "NAV_DEPTH",
      "NAV_SPEED", "NAV_YAW", "NAV_PITCH", "NAV_ROLL",
      "DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_DEPTH"};

  const std::string package_path = ros::package::getPath("ros_helm");
  const std::string base_dir = package_path.empty() ? "log"
                                                    : package_path + "/log";

  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buffer;
  localtime_r(&now_time, &tm_buffer);

  std::ostringstream folder_name;
  folder_name << std::put_time(&tm_buffer, "%Y_%m_%d_%H_%M") << "_log";

  log_directory_ = base_dir + "/" + folder_name.str();

  auto create_directory = [](const std::string &path) {
    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST)
    {
      ROS_ERROR_STREAM("Failed to create directory: " << path
                       << " error: " << std::strerror(errno));
      return false;
    }
    return true;
  };

  if (!create_directory(base_dir))
    return false;
  if (!create_directory(log_directory_))
    return false;

  for (const auto &key : kLogKeys)
  {
    const std::string file_path = log_directory_ + "/" + key + ".txt";
    std::ofstream stream(file_path, std::ios::trunc);
    if (!stream.is_open())
    {
      ROS_ERROR_STREAM("Failed to open log file: " << file_path);
      return false;
    }
    log_streams_[key] = std::move(stream);
  }

  ROS_INFO_STREAM("[ros_bridge] logging to " << log_directory_);
  return true;
}

void RosBridge::logValue(const std::string &name, double value,
                         const ros::Time &stamp)
{
  constexpr std::size_t kFlushLineInterval = 100;
  const ros::Duration kFlushPeriod(0.1);

  std::lock_guard<std::mutex> guard(log_mutex_);
  auto it = log_streams_.find(name);
  if (it == log_streams_.end() || !it->second.is_open())
    return;

  it->second << name << ' ' << std::fixed << std::setprecision(3) << value
             << ' ' << stamp.toSec() << '\n';

  ++log_lines_since_flush_;
  ros::Time flush_time = stamp;
  if (flush_time.isZero())
    flush_time = ros::Time::now();
  if (last_log_flush_time_.isZero())
    last_log_flush_time_ = flush_time;

  if (log_lines_since_flush_ >= kFlushLineInterval ||
      (flush_time - last_log_flush_time_) >= kFlushPeriod)
  {
    it->second.flush();
    log_lines_since_flush_ = 0;
    last_log_flush_time_ = flush_time;
  }
}
