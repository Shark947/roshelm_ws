#include "ros_behavior_tree/bt_nodes.hpp"
#include "ros_behavior_tree/nav_subscriber.hpp"

#include <ros/package.h>
#include <ros/ros.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
namespace
{
std::string defaultTopic(const std::string &vehicle, const std::string &suffix)
{
  if (vehicle.empty())
    return suffix;
  return "/" + vehicle + "/" + suffix;
}
}

namespace ros_behavior_tree
{

class BehaviorTreeNode
{
public:
  BehaviorTreeNode()
      : private_nh_("~")
  {
    private_nh_.param("vehicle_name", vehicle_name_, std::string("auh"));
    private_nh_.param("loop_frequency", loop_frequency_, 5.0);
    private_nh_.param("nav_log_period", nav_log_period_, 0.05);

    double trigger_hold_time = 0.0;
    private_nh_.param("trigger_hold_time", trigger_hold_time, 1.0);
    store_.setTriggerHold(ros::Duration(trigger_hold_time));
    bool latch_deploy = false;
    private_nh_.param("latch_deploy", latch_deploy, false);
    store_.setDeployLatch(latch_deploy);

    NavTopics topics;
    private_nh_.param("ros_heading_topic", topics.ros_heading,
                      defaultTopic(vehicle_name_, "current_heading"));
    private_nh_.param("ros_speed_topic", topics.ros_speed,
                      defaultTopic(vehicle_name_, "current_speed"));
    private_nh_.param("ros_depth_topic", topics.ros_depth,
                      defaultTopic(vehicle_name_, "current_depth"));
    private_nh_.param("ros_yaw_topic", topics.ros_yaw,
                      defaultTopic(vehicle_name_, "current_yaw"));
    private_nh_.param("ros_pitch_topic", topics.ros_pitch,
                      defaultTopic(vehicle_name_, "current_pitch"));
    private_nh_.param("ros_roll_topic", topics.ros_roll,
                      defaultTopic(vehicle_name_, "current_roll"));
    private_nh_.param("ros_x_topic", topics.ros_x,
                      defaultTopic(vehicle_name_, "current_x"));
    private_nh_.param("ros_y_topic", topics.ros_y,
                      defaultTopic(vehicle_name_, "current_y"));
    private_nh_.param("deploy_topic", topics.deploy,
                      defaultTopic(vehicle_name_, "DEPLOY"));
    private_nh_.param("return_topic", topics.return_home,
                      defaultTopic(vehicle_name_, "RETURN"));
    private_nh_.param("speed_trigger_topic", topics.speed_trigger,
                      defaultTopic(vehicle_name_, "SPD"));

    private_nh_.param("nav_heading_topic", topics.nav_heading,
                      defaultTopic(vehicle_name_, "NAV_HEADING"));
    private_nh_.param("nav_speed_topic", topics.nav_speed,
                      defaultTopic(vehicle_name_, "NAV_SPEED"));
    private_nh_.param("nav_depth_topic", topics.nav_depth,
                      defaultTopic(vehicle_name_, "NAV_DEPTH"));
    private_nh_.param("nav_yaw_topic", topics.nav_yaw,
                      defaultTopic(vehicle_name_, "NAV_YAW"));
    private_nh_.param("nav_pitch_topic", topics.nav_pitch,
                      defaultTopic(vehicle_name_, "NAV_PITCH"));
    private_nh_.param("nav_roll_topic", topics.nav_roll,
                      defaultTopic(vehicle_name_, "NAV_ROLL"));
    private_nh_.param("nav_x_topic", topics.nav_x,
                      defaultTopic(vehicle_name_, "NAV_X"));
    private_nh_.param("nav_y_topic", topics.nav_y,
                      defaultTopic(vehicle_name_, "NAV_Y"));

    ROS_INFO_STREAM("[ros_behavior_tree] vehicle_name=" << vehicle_name_);
    ROS_INFO_STREAM("[ros_behavior_tree] ROS topics: heading=" << topics.ros_heading
                    << " speed=" << topics.ros_speed
                    << " depth=" << topics.ros_depth
                    << " yaw=" << topics.ros_yaw
                    << " pitch=" << topics.ros_pitch
                    << " roll=" << topics.ros_roll
                    << " x=" << topics.ros_x
                    << " y=" << topics.ros_y
                    << " deploy=" << topics.deploy
                    << " return=" << topics.return_home
                    << " speed_trigger=" << topics.speed_trigger);
    ROS_INFO_STREAM("[ros_behavior_tree] NAV topics: heading=" << topics.nav_heading
                    << " speed=" << topics.nav_speed
                    << " depth=" << topics.nav_depth
                    << " yaw=" << topics.nav_yaw
                    << " pitch=" << topics.nav_pitch
                    << " roll=" << topics.nav_roll
                    << " x=" << topics.nav_x
                    << " y=" << topics.nav_y);

    nav_subscriber_ = std::make_shared<NavDataSubscriber>(nh_, topics, store_);
    manager_ = std::make_shared<BehaviorTreeManager>(private_nh_, &store_);

    if (nav_log_period_ > 0.0)
    {
      log_ready_ = setupLogDirectory();
    }
  }

  void spin()
  {
    ros::Rate rate(loop_frequency_);
    while (ros::ok())
    {
      ros::spinOnce();
      if (manager_)
        manager_->tick();
      logNavIfNeeded();
      rate.sleep();
    }
  }

private:
  bool setupLogDirectory()
  {
    static const std::array<std::string, 11> kLogKeys = {
        "NAV_X", "NAV_Y", "NAV_HEADING", "NAV_DEPTH",
        "NAV_SPEED", "NAV_YAW", "NAV_PITCH", "NAV_ROLL",
        "DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_DEPTH"};

    const std::string package_path = ros::package::getPath("ros_behavior_tree");
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

    ROS_INFO_STREAM("[ros_behavior_tree] logging to " << log_directory_);
    return true;
  }

  void logNavIfNeeded()
  {
    if (!log_ready_ || nav_log_period_ <= 0.0)
      return;

    const ros::Time now = ros::Time::now();
    if (!last_nav_log_.isZero() &&
        (now - last_nav_log_).toSec() < nav_log_period_)
      return;

    last_nav_log_ = now;

    auto log_value = [&](const std::string &key, bool has_value,
                         double value) {
      auto it = log_streams_.find(key);
      if (!has_value || it == log_streams_.end() || !it->second.is_open())
        return;
      it->second << key << ' ' << std::fixed << std::setprecision(3) << value
                 << ' ' << now.toSec() << '\n';
      it->second.flush();
    };

    double heading = 0.0;
    double speed = 0.0;
    double depth = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double x = 0.0;
    double y = 0.0;
    const ros::Duration timeout(0.0);

    const bool has_heading = store_.preferredHeading(heading, timeout);
    const bool has_speed = store_.preferredSpeed(speed, timeout);
    const bool has_depth = store_.preferredDepth(depth, timeout);
    const bool has_yaw = store_.preferredYaw(yaw, timeout);
    const bool has_pitch = store_.preferredPitch(pitch, timeout);
    const bool has_roll = store_.preferredRoll(roll, timeout);
    const bool has_x = store_.preferredX(x, timeout);
    const bool has_y = store_.preferredY(y, timeout);

    log_value("NAV_HEADING", has_heading, heading);
    log_value("NAV_SPEED", has_speed, speed);
    log_value("NAV_DEPTH", has_depth, depth);
    log_value("NAV_YAW", has_yaw, yaw);
    log_value("NAV_PITCH", has_pitch, pitch);
    log_value("NAV_ROLL", has_roll, roll);
    log_value("NAV_X", has_x, x);
    log_value("NAV_Y", has_y, y);

    const std::map<std::string, double> desired_values =
        manager_ ? manager_->desiredValues() : std::map<std::string, double>{};
    auto log_desired = [&](const std::string &key) {
      auto it = desired_values.find(key);
      if (it == desired_values.end())
        return;
      log_value(key, true, it->second);
    };
    log_desired("DESIRED_HEADING");
    log_desired("DESIRED_SPEED");
    log_desired("DESIRED_DEPTH");

    auto format_values = [&](const std::map<std::string, double> &values,
                             const std::string &prefix) {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3);
      bool first = true;
      for (const auto &entry : values)
      {
        if (!prefix.empty() && entry.first.rfind(prefix, 0) != 0)
          continue;
        if (!first)
          stream << ", ";
        stream << entry.first << "=" << entry.second;
        first = false;
      }
      return stream.str();
    };

    std::ostringstream status;
    status << std::fixed << std::setprecision(3);
    status << "[ros_behavior_tree] nav_log t=" << now.toSec();
    const std::string desired_text =
        format_values(desired_values, "DESIRED_");
    status << " DESIRED={" << (desired_text.empty() ? "none" : desired_text)
           << "}";
    if (has_heading)
      status << " NAV_HEADING=" << heading;
    if (has_speed)
      status << " NAV_SPEED=" << speed;
    if (has_depth)
      status << " NAV_DEPTH=" << depth;
    if (has_yaw)
      status << " NAV_YAW=" << yaw;
    if (has_pitch)
      status << " NAV_PITCH=" << pitch;
    if (has_roll)
      status << " NAV_ROLL=" << roll;
    if (has_x)
      status << " NAV_X=" << x;
    if (has_y)
      status << " NAV_Y=" << y;
    ROS_INFO_STREAM(status.str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string vehicle_name_;
  double loop_frequency_{5.0};
  double nav_log_period_{0.05};

  NavDataStore store_;
  std::shared_ptr<NavDataSubscriber> nav_subscriber_;
  std::shared_ptr<BehaviorTreeManager> manager_;
  bool log_ready_{false};
  ros::Time last_nav_log_;
  std::string log_directory_;
  std::map<std::string, std::ofstream> log_streams_;
};

}  // namespace ros_behavior_tree

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_behavior_tree");
  ros_behavior_tree::BehaviorTreeNode node;
  node.spin();
  return 0;
}
