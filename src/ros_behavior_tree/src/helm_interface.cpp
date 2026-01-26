#include "ros_behavior_tree/helm_interface.hpp"

#include <cmath>
#include <sstream>

namespace
{
bool parseDomainValue(const std::string &value, double &low, double &high,
                      unsigned int &points)
{
  std::stringstream stream(value);
  std::string token;
  if (!std::getline(stream, token, ':'))
    return false;
  low = std::stod(token);
  if (!std::getline(stream, token, ':'))
    return false;
  high = std::stod(token);
  if (!std::getline(stream, token, ':'))
    return false;
  points = static_cast<unsigned int>(std::stoul(token));
  return true;
}

double wrapHeading(double heading)
{
  double wrapped = std::fmod(heading, 360.0);
  if (wrapped < 0.0)
    wrapped += 360.0;
  return wrapped;
}
}  // namespace

namespace ros_behavior_tree
{

HelmInterface::HelmInterface(const ros::NodeHandle &nh,
                             const NavDataStore *store)
    : nh_(nh), store_(store)
{
}

bool HelmInterface::initialize()
{
  nh_.param("vehicle_name", vehicle_name_, std::string("auh"));
  nh_.param("nav_timeout", nav_timeout_, 0.0);

  nh_.param("desired_heading_topic", desired_heading_topic_,
            std::string("/" + vehicle_name_ + "/desired_heading"));
  nh_.param("desired_speed_topic", desired_speed_topic_,
            std::string("/" + vehicle_name_ + "/desired_speed"));
  nh_.param("desired_depth_topic", desired_depth_topic_,
            std::string("/" + vehicle_name_ + "/desired_depth"));
  nh_.param("deploy_topic", deploy_topic_,
            std::string("/" + vehicle_name_ + "/DEPLOY"));
  nh_.param("return_topic", return_topic_,
            std::string("/" + vehicle_name_ + "/RETURN"));
  nh_.param("mission_topic", mission_topic_,
            std::string("/" + vehicle_name_ + "/MISSION"));

  std::string course_domain;
  std::string speed_domain;
  std::string depth_domain;
  nh_.param("domain_course", course_domain, std::string("0:359:360"));
  nh_.param("domain_speed", speed_domain, std::string("0:4:41"));
  nh_.param("domain_depth", depth_domain, std::string("0:1000:1001"));

  if (!parseDomain("course", course_domain) ||
      !parseDomain("speed", speed_domain) ||
      !parseDomain("depth", depth_domain))
  {
    ROS_ERROR_STREAM("[ros_behavior_tree] Failed to parse domain config");
    return false;
  }

  helm_engine_.reset(new HelmEngine(domain_, &info_buffer_, &ledger_snap_));
  behavior_set_.setDomain(domain_);
  behavior_set_.connectInfoBuffer(&info_buffer_);
  behavior_set_.connectLedgerSnap(&ledger_snap_);
  behavior_set_.setOwnship(vehicle_name_);

  desired_heading_pub_ = nh_.advertise<std_msgs::Float64>(
      desired_heading_topic_, 10);
  desired_speed_pub_ = nh_.advertise<std_msgs::Float64>(
      desired_speed_topic_, 10);
  desired_depth_pub_ = nh_.advertise<std_msgs::Float64>(
      desired_depth_topic_, 10);
  deploy_pub_ = nh_.advertise<std_msgs::Bool>(deploy_topic_, 10, true);
  return_pub_ = nh_.advertise<std_msgs::Bool>(return_topic_, 10, true);
  mission_pub_ = nh_.advertise<std_msgs::String>(mission_topic_, 10, true);

  return true;
}

bool HelmInterface::parseDomain(const std::string &name,
                                const std::string &value)
{
  double low = 0.0;
  double high = 0.0;
  unsigned int points = 0;
  if (!parseDomainValue(value, low, high, points))
    return false;
  return domain_.addDomain(name, low, high, points);
}

bool HelmInterface::updateInfoBuffer()
{
  if (!store_)
    return false;

  ros::Duration timeout(nav_timeout_);
  double heading = 0.0;
  double speed = 0.0;
  double depth = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;

  if (!store_->preferredHeading(heading, timeout) ||
      !store_->preferredSpeed(speed, timeout) ||
      !store_->preferredDepth(depth, timeout) ||
      !store_->preferredX(x, timeout) ||
      !store_->preferredY(y, timeout))
  {
    return false;
  }

  const ros::Time now = ros::Time::now();
  if (start_time_.isZero())
    start_time_ = now;

  info_buffer_.setCurrTime(now.toSec());
  info_buffer_.setStartTime(start_time_.toSec());

  info_buffer_.setValue("NAV_HEADING", wrapHeading(heading), now.toSec());
  info_buffer_.setValue("NAV_SPEED", speed, now.toSec());
  info_buffer_.setValue("NAV_DEPTH", depth, now.toSec());
  info_buffer_.setValue("NAV_X", x, now.toSec());
  info_buffer_.setValue("NAV_Y", y, now.toSec());

  if (store_->preferredYaw(yaw, timeout))
    info_buffer_.setValue("NAV_YAW", yaw, now.toSec());
  if (store_->preferredPitch(pitch, timeout))
    info_buffer_.setValue("NAV_PITCH", pitch, now.toSec());
  if (store_->preferredRoll(roll, timeout))
    info_buffer_.setValue("NAV_ROLL", roll, now.toSec());

  return true;
}

bool HelmInterface::solveForBehavior(IvPBehavior &behavior)
{
  if (!helm_engine_)
    return false;

  auto active_it = behavior_active_.find(&behavior);
  if (active_it == behavior_active_.end() || !active_it->second)
  {
    behavior_active_[&behavior] = true;
    behavior_set_dirty_ = true;
  }

  if (!updateInfoBuffer())
    return false;

  if (behavior_set_dirty_)
    rebuildBehaviorSet();

  behavior_set_.setCurrTime(info_buffer_.getCurrTime());

  const HelmReport report =
      helm_engine_->determineNextDecision(&behavior_set_,
                                          info_buffer_.getCurrTime());
  publishDesired(report);
  return true;
}

void HelmInterface::deactivateBehavior(IvPBehavior &behavior)
{
  auto it = behavior_active_.find(&behavior);
  if (it != behavior_active_.end() && it->second)
  {
    it->second = false;
    behavior_set_dirty_ = true;
  }
}

std::map<std::string, double> HelmInterface::desiredValues() const
{
  return desired_values_;
}

void HelmInterface::publishDesired(const HelmReport &report)
{
  std::map<std::string, double> desired_snapshot;
  auto publish_scalar = [&](const std::string &key, ros::Publisher &pub,
                            bool rotate_heading) {
    if (!report.hasDecision(key))
      return;
    std_msgs::Float64 msg;
    msg.data = report.getDecision(key);
    if (rotate_heading)
      msg.data = wrapHeading(90.0 - msg.data);
    if (pub)
      pub.publish(msg);
    desired_snapshot[key] = msg.data;
  };

  publish_scalar("course", desired_heading_pub_, true);
  publish_scalar("speed", desired_speed_pub_, false);
  publish_scalar("depth", desired_depth_pub_, false);

  desired_values_.clear();
  if (!desired_snapshot.empty())
  {
    auto heading = desired_snapshot.find("course");
    if (heading != desired_snapshot.end())
      desired_values_["DESIRED_HEADING"] = heading->second;
    auto speed = desired_snapshot.find("speed");
    if (speed != desired_snapshot.end())
      desired_values_["DESIRED_SPEED"] = speed->second;
    auto depth = desired_snapshot.find("depth");
    if (depth != desired_snapshot.end())
      desired_values_["DESIRED_DEPTH"] = depth->second;
  }
}

void HelmInterface::publishMissionComplete(const std::string &value)
{
  if (deploy_pub_)
  {
    std_msgs::Bool msg;
    msg.data = false;
    deploy_pub_.publish(msg);
  }
  if (return_pub_)
  {
    std_msgs::Bool msg;
    msg.data = false;
    return_pub_.publish(msg);
  }
  if (mission_pub_)
  {
    std_msgs::String msg;
    msg.data = value;
    mission_pub_.publish(msg);
  }
}

void HelmInterface::rebuildBehaviorSet()
{
  behavior_set_.clearBehaviors();
  for (const auto &entry : behavior_active_)
  {
    if (!entry.second)
      continue;
    entry.first->setInfoBuffer(&info_buffer_);
    entry.first->setLedgerSnap(&ledger_snap_);
    behavior_set_.addBehavior(entry.first);
  }
  behavior_set_.connectInfoBuffer(&info_buffer_);
  behavior_set_.connectLedgerSnap(&ledger_snap_);
  behavior_set_dirty_ = false;
}

}  // namespace ros_behavior_tree
