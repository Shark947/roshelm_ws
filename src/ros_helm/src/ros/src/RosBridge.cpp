#include "RosBridge.h"

#include <array>
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

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double normalizeAngle360(double angle_deg)
{
  double value = std::fmod(angle_deg, 360.0);
  if (value < 0.0)
    value += 360.0;
  return value;
}

double normalizeAngle180(double angle_deg)
{
  double value = std::fmod(angle_deg + 180.0, 360.0);
  if (value < 0.0) value += 360.0;
  return value - 180.0;
}

double headingFromRotNED(const tf::Matrix3x3 &R_m)
{
  // body x-axis in world = R * [1,0,0]
  const tf::Vector3 f = R_m * tf::Vector3(1.0, 0.0, 0.0);
  const double chi = std::atan2(f.getY(), f.getX()) * kRadToDeg; // atan2(East, North)
  return normalizeAngle360(chi);
}

double yawFromRotENU(const tf::Matrix3x3 &R_r)
{
  const tf::Vector3 f = R_r * tf::Vector3(1.0, 0.0, 0.0);
  const double psi = std::atan2(f.getY(), f.getX()) * kRadToDeg; // atan2(North, East)
  return normalizeAngle360(psi);
}

tf::Matrix3x3 rosToMoosMatrix()
{
  return tf::Matrix3x3(0.0, 1.0, 0.0,
                       1.0, 0.0, 0.0,
                       0.0, 0.0, -1.0);
}

struct RpyDeg
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

RpyDeg convertRosToMoosRpy(double heading_deg, double pitch_deg, double roll_deg)
{
  tf::Matrix3x3 ros_rot;
  ros_rot.setRPY(roll_deg * kDegToRad,
                 pitch_deg * kDegToRad,
                 heading_deg * kDegToRad);

  tf::Matrix3x3 moos_rot = rosToMoosMatrix() * ros_rot;

  double moos_roll = 0.0, moos_pitch = 0.0, moos_yaw_unused = 0.0;
  moos_rot.getRPY(moos_roll, moos_pitch, moos_yaw_unused);

  const double heading_moos = headingFromRotNED(moos_rot);

  return {normalizeAngle180(moos_roll * kRadToDeg),
          normalizeAngle180(moos_pitch * kRadToDeg),
          heading_moos};
}

RpyDeg convertMoosToRosRpy(double heading_deg, double pitch_deg, double roll_deg)
{
  tf::Matrix3x3 moos_rot;
  moos_rot.setRPY(roll_deg * kDegToRad,
                  pitch_deg * kDegToRad,
                  heading_deg * kDegToRad);

  tf::Matrix3x3 ros_rot = rosToMoosMatrix().transpose() * moos_rot;

  double ros_roll = 0.0, ros_pitch = 0.0, ros_yaw_unused = 0.0;
  ros_rot.getRPY(ros_roll, ros_pitch, ros_yaw_unused);

  const double yaw_ros = yawFromRotENU(ros_rot);

  return {normalizeAngle180(ros_roll * kRadToDeg),
          normalizeAngle180(ros_pitch * kRadToDeg),
          yaw_ros};
}

double convertMoosDepthToRos(double moos_depth)
{
  const tf::Vector3 moos_pos(0.0, 0.0, moos_depth);
  const tf::Vector3 ros_pos = rosToMoosMatrix().transpose() * moos_pos;
  return ros_pos.getZ();
}

double convertRosSpeedToMoos(double ros_speed)
{
  const tf::Vector3 ros_vel(ros_speed, 0.0, 0.0);
  const tf::Vector3 moos_vel = rosToMoosMatrix() * ros_vel;
  return std::hypot(moos_vel.getX(), moos_vel.getY());
}

double convertMoosSpeedToRos(double moos_speed)
{
  const tf::Vector3 moos_vel(moos_speed, 0.0, 0.0);
  const tf::Vector3 ros_vel = rosToMoosMatrix().transpose() * moos_vel;
  return std::hypot(ros_vel.getX(), ros_vel.getY());
}

tf::Vector3 convertRosPositionToMoos(double x, double y, double z)
{
  const tf::Vector3 ros_pos(x, y, z);
  return rosToMoosMatrix() * ros_pos;
}
}

RosBridge::RosBridge(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                     const RosNodeConfig &config)
    : nh_(nh), private_nh_(private_nh), config_(config)
{
}

bool RosBridge::initialize()
{
  if (!setupLogDirectory())
    return false;

  heading_sub_ = subscribeCurrent(config_.current_heading_topic, "NAV_HEADING");
  speed_sub_ = subscribeCurrent(config_.current_speed_topic, "NAV_SPEED");
  depth_sub_ = subscribeCurrent(config_.current_depth_topic, "NAV_DEPTH");
  x_sub_ = subscribeCurrent(config_.current_x_topic, "NAV_X");
  y_sub_ = subscribeCurrent(config_.current_y_topic, "NAV_Y");
  pitch_sub_ = subscribeCurrent(config_.current_pitch_topic, "NAV_PITCH");
  roll_sub_ = subscribeCurrent(config_.current_roll_topic, "NAV_ROLL");

  deploy_sub_ = subscribeBoolean(config_.deploy_topic, "DEPLOY");
  return_sub_ = subscribeBoolean(config_.return_topic, "RETURN");

  deploy_pub_ = nh_.advertise<std_msgs::Bool>(config_.deploy_topic, 1, true);
  return_pub_ = nh_.advertise<std_msgs::Bool>(config_.return_topic, 1, true);

  std_msgs::Bool default_msg;
  default_msg.data = config_.deploy_default;
  deploy_pub_.publish(default_msg);
  enqueueBoolValue("DEPLOY", default_msg.data, ros::Time::now());

  default_msg.data = config_.return_default;
  return_pub_.publish(default_msg);
  enqueueBoolValue("RETURN", default_msg.data, ros::Time::now());

  desired_scalar_pubs_["DESIRED_HEADING"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_heading_topic, 10);
  desired_scalar_pubs_["DESIRED_SPEED"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_speed_topic, 10);
  desired_scalar_pubs_["DESIRED_DEPTH"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_depth_topic, 10);
  desired_scalar_pubs_["DESIRED_PITCH"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_pitch_topic, 10);
  desired_scalar_pubs_["DESIRED_ROLL"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_roll_topic, 10);

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

ros::Subscriber RosBridge::subscribeBoolean(const std::string &topic,
                                            const std::string &key)
{
  if (topic.empty())
    return {};

  return nh_.subscribe<std_msgs::Bool>(
      topic, 10,
      [this, key](const std_msgs::Bool::ConstPtr &msg) {
        this->booleanCallback(msg, key);
      });
}

void RosBridge::currentValueCallback(
    const common_msgs::Float64Stamped::ConstPtr &msg,
    const std::string &nav_key)
{
  if (nav_key == "NAV_HEADING" || nav_key == "NAV_PITCH" || nav_key == "NAV_ROLL")
  {
    if (nav_key == "NAV_HEADING")
    {
      ros_orientation_.heading_deg = msg->data;
      ros_orientation_.heading_stamp = msg->header.stamp;
      ros_orientation_.has_heading = true;
    }
    else if (nav_key == "NAV_PITCH")
    {
      ros_orientation_.pitch_deg = msg->data;
      ros_orientation_.pitch_stamp = msg->header.stamp;
      ros_orientation_.has_pitch = true;
    }
    else
    {
      ros_orientation_.roll_deg = msg->data;
      ros_orientation_.roll_stamp = msg->header.stamp;
      ros_orientation_.has_roll = true;
    }

    if (ros_orientation_.has_heading && ros_orientation_.has_pitch &&
        ros_orientation_.has_roll)
    {
      ros::Time min_stamp = ros_orientation_.heading_stamp;
      ros::Time max_stamp = ros_orientation_.heading_stamp;
      auto update_minmax = [](const ros::Time &stamp, ros::Time &min_val,
                              ros::Time &max_val) {
        if (stamp < min_val)
          min_val = stamp;
        if (stamp > max_val)
          max_val = stamp;
      };
      update_minmax(ros_orientation_.pitch_stamp, min_stamp, max_stamp);
      update_minmax(ros_orientation_.roll_stamp, min_stamp, max_stamp);
      const double span = (max_stamp - min_stamp).toSec();
      if (span > config_.orientation_sync_tolerance)
      {
        ROS_WARN_THROTTLE(1.0,
                          "[ros_bridge] orientation sample span %.3fs exceeds "
                          "tolerance %.3fs, waiting for synced samples",
                          span, config_.orientation_sync_tolerance);
        OrientationCache fresh;
        if (nav_key == "NAV_HEADING")
        {
          fresh.heading_deg = msg->data;
          fresh.heading_stamp = msg->header.stamp;
          fresh.has_heading = true;
        }
        else if (nav_key == "NAV_PITCH")
        {
          fresh.pitch_deg = msg->data;
          fresh.pitch_stamp = msg->header.stamp;
          fresh.has_pitch = true;
        }
        else
        {
          fresh.roll_deg = msg->data;
          fresh.roll_stamp = msg->header.stamp;
          fresh.has_roll = true;
        }
        ros_orientation_ = fresh;
        return;
      }
      const auto moos_rpy = convertRosToMoosRpy(
          ros_orientation_.heading_deg,
          ros_orientation_.pitch_deg,
          ros_orientation_.roll_deg);
      enqueueNavValue("NAV_HEADING", moos_rpy.yaw, msg->header.stamp);
      enqueueNavValue("NAV_PITCH", moos_rpy.pitch, msg->header.stamp);
      enqueueNavValue("NAV_ROLL", moos_rpy.roll, msg->header.stamp);
    }
    return;
  }

  if (nav_key == "NAV_X" || nav_key == "NAV_Y" || nav_key == "NAV_DEPTH")
  {
    if (nav_key == "NAV_X")
    {
      ros_position_.x = msg->data;
      ros_position_.has_x = true;
    }
    else if (nav_key == "NAV_Y")
    {
      ros_position_.y = msg->data;
      ros_position_.has_y = true;
    }
    else
    {
      ros_position_.z = msg->data;
      ros_position_.has_z = true;
    }

    if (ros_position_.has_x && ros_position_.has_y && ros_position_.has_z)
    {
      const tf::Vector3 moos_pos = convertRosPositionToMoos(
          ros_position_.x, ros_position_.y, ros_position_.z);
      enqueueNavValue("NAV_X", moos_pos.getX(), msg->header.stamp);
      enqueueNavValue("NAV_Y", moos_pos.getY(), msg->header.stamp);
      enqueueNavValue("NAV_DEPTH", moos_pos.getZ(), msg->header.stamp);
    }
    return;
  }

  if (nav_key == "NAV_SPEED")
  {
    const double moos_speed = convertRosSpeedToMoos(msg->data);
    enqueueNavValue(nav_key, moos_speed, msg->header.stamp);
    return;
  }

  enqueueNavValue(nav_key, msg->data, msg->header.stamp);
}

void RosBridge::booleanCallback(const std_msgs::Bool::ConstPtr &msg,
                                const std::string &key)
{
  enqueueBoolValue(key, msg->data, ros::Time::now());
}

void RosBridge::deliverPending(HelmIvP &helm)
{
  std::deque<HelmMsg> mail;
  {
    std::lock_guard<std::mutex> guard(mail_mutex_);
    mail.swap(pending_mail_);
  }

  if (mail.empty())
    return;

  std::list<HelmMsg> pending(mail.begin(), mail.end());
  helm.OnNewMail(pending);
}

void RosBridge::publishDesired(const HelmIvP &helm)
{
  const auto desired = collectDesiredDoubles(helm);
  if (desired.empty())
    return;

  const auto now = ros::Time::now();

  const bool has_heading = desired.count("DESIRED_HEADING") > 0;
  const bool has_pitch = desired.count("DESIRED_PITCH") > 0;
  const bool has_roll = desired.count("DESIRED_ROLL") > 0;

  if (has_heading || has_pitch || has_roll)
  {
    const double heading = has_heading ? desired.at("DESIRED_HEADING") : 0.0;
    const double pitch = has_pitch ? desired.at("DESIRED_PITCH") : 0.0;
    const double roll = has_roll ? desired.at("DESIRED_ROLL") : 0.0;
    const auto ros_rpy = convertMoosToRosRpy(heading, pitch, roll);

    if (has_heading && desired_scalar_pubs_.count("DESIRED_HEADING"))
    {
      std_msgs::Float64 msg;
      msg.data = ros_rpy.yaw;
      desired_scalar_pubs_.at("DESIRED_HEADING").publish(msg);
      logValue("DESIRED_HEADING", msg.data, now);
    }
    if (has_pitch && desired_scalar_pubs_.count("DESIRED_PITCH"))
    {
      std_msgs::Float64 msg;
      msg.data = ros_rpy.pitch;
      desired_scalar_pubs_.at("DESIRED_PITCH").publish(msg);
      logValue("DESIRED_PITCH", msg.data, now);
    }
    if (has_roll && desired_scalar_pubs_.count("DESIRED_ROLL"))
    {
      std_msgs::Float64 msg;
      msg.data = ros_rpy.roll;
      desired_scalar_pubs_.at("DESIRED_ROLL").publish(msg);
      logValue("DESIRED_ROLL", msg.data, now);
    }
  }

  if (desired.count("DESIRED_SPEED") > 0 &&
      desired_scalar_pubs_.count("DESIRED_SPEED"))
  {
    std_msgs::Float64 msg;
    msg.data = convertMoosSpeedToRos(desired.at("DESIRED_SPEED"));
    desired_scalar_pubs_.at("DESIRED_SPEED").publish(msg);
    logValue("DESIRED_SPEED", msg.data, now);
  }

  if (desired.count("DESIRED_DEPTH") > 0 &&
      desired_scalar_pubs_.count("DESIRED_DEPTH"))
  {
    std_msgs::Float64 msg;
    msg.data = convertMoosDepthToRos(desired.at("DESIRED_DEPTH"));
    desired_scalar_pubs_.at("DESIRED_DEPTH").publish(msg);
    logValue("DESIRED_DEPTH", msg.data, now);
  }
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
  static const std::array<std::string, 12> kLogKeys = {
      "NAV_X", "NAV_Y", "NAV_HEADING", "NAV_DEPTH",
      "NAV_SPEED", "NAV_PITCH", "NAV_ROLL", "DESIRED_HEADING",
      "DESIRED_SPEED", "DESIRED_DEPTH", "DESIRED_PITCH", "DESIRED_ROLL"};

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
  std::lock_guard<std::mutex> guard(log_mutex_);
  auto it = log_streams_.find(name);
  if (it == log_streams_.end() || !it->second.is_open())
    return;

  it->second << name << ' ' << std::fixed << std::setprecision(3) << value
             << ' ' << stamp.toSec() << std::endl;
}
