#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#include <ros/ros.h>
#include <common_msgs/Float64Stamped.h>
#include <docking_optical_msgs/OpticalMeasurement.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "docking_nav/docking_nav_server.h"

namespace
{
struct DockingNavTopics
{
  std::string optical_measurement_topic{"/docking/optical_measurement"};
  std::string phase_topic{"/docking/phase"};
  std::string optical_xy_topic{"/docking/optical_xy"};
  std::string optical_feedback_topic{"/docking/optical_feedback"};
  std::string mode_state_topic{"/docking/mode_state"};
  std::string nav_heading_topic{"/auh/NAV_HEADING"};
  std::string nav_depth_topic{"/auh/NAV_DEPTH"};
  std::string nav_pitch_topic{"/auh/NAV_PITCH"};
  std::string nav_roll_topic{"/auh/NAV_ROLL"};
  std::string desired_speed_topic{"/auh/desired_speed"};
  std::string dock_depth_topic{"/docking/dock_depth"};
};

struct DockingCommandTopics
{
  std::string mode_topic{"/docking/mode"};
  std::string stationing_topic{"/docking/stationing"};
  std::string constheight_topic{"/docking/constheight"};
  std::string dockdepth_update_topic{"/docking/dockdepth_update"};
  std::string dockhdg_updates_topic{"/docking/dockhdg_updates"};
  std::string docking_falling_topic{"/docking/docking_falling"};
  std::string manual_override_topic{"/docking/manual_override"};
  std::string docking_failed_topic{"/docking/docking_failed"};
};

DockingNavTopics loadDockingNavTopics(ros::NodeHandle &private_nh)
{
  DockingNavTopics topics;
  private_nh.param("optical_measurement_topic", topics.optical_measurement_topic,
                   topics.optical_measurement_topic);
  private_nh.param("phase_topic", topics.phase_topic, topics.phase_topic);
  private_nh.param("optical_xy_topic", topics.optical_xy_topic,
                   topics.optical_xy_topic);
  private_nh.param("optical_feedback_topic", topics.optical_feedback_topic,
                   topics.optical_feedback_topic);
  private_nh.param("mode_state_topic", topics.mode_state_topic,
                   topics.mode_state_topic);
  private_nh.param("nav_heading_topic", topics.nav_heading_topic,
                   topics.nav_heading_topic);
  private_nh.param("nav_depth_topic", topics.nav_depth_topic,
                   topics.nav_depth_topic);
  private_nh.param("nav_pitch_topic", topics.nav_pitch_topic,
                   topics.nav_pitch_topic);
  private_nh.param("nav_roll_topic", topics.nav_roll_topic,
                   topics.nav_roll_topic);
  private_nh.param("desired_speed_topic", topics.desired_speed_topic,
                   topics.desired_speed_topic);
  private_nh.param("dock_depth_topic", topics.dock_depth_topic,
                   topics.dock_depth_topic);
  return topics;
}

DockingCommandTopics loadDockingCommandTopics(ros::NodeHandle &private_nh)
{
  DockingCommandTopics topics;
  private_nh.param("mode_topic", topics.mode_topic, topics.mode_topic);
  private_nh.param("stationing_topic", topics.stationing_topic,
                   topics.stationing_topic);
  private_nh.param("constheight_topic", topics.constheight_topic,
                   topics.constheight_topic);
  private_nh.param("dockdepth_update_topic", topics.dockdepth_update_topic,
                   topics.dockdepth_update_topic);
  private_nh.param("dockhdg_updates_topic", topics.dockhdg_updates_topic,
                   topics.dockhdg_updates_topic);
  private_nh.param("docking_falling_topic", topics.docking_falling_topic,
                   topics.docking_falling_topic);
  private_nh.param("manual_override_topic", topics.manual_override_topic,
                   topics.manual_override_topic);
  private_nh.param("docking_failed_topic", topics.docking_failed_topic,
                   topics.docking_failed_topic);
  return topics;
}
}  // namespace

class DockingNavMain
{
public:
  DockingNavMain(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
      : nh_(nh),
        private_nh_(private_nh),
        topics_(loadDockingNavTopics(private_nh)),
        command_topics_(loadDockingCommandTopics(private_nh))
  {
    server_.setCommandCallbacks(
        [this](const std::string &key, bool value) {
          this->publishBoolCommand(key, value);
        },
        [this](const std::string &key, const std::string &value) {
          this->publishStringCommand(key, value);
        });
    server_.setDebugLogCallback([this](const std::string &message) {
      this->logDebug(message);
    });
  }

  bool initialize()
  {
    private_nh_.param("debug_log_path", debug_log_path_,
                      std::string("/tmp/docking_nav_debug.txt"));
    debug_log_.open(debug_log_path_, std::ios::app);
    if (debug_log_.is_open())
    {
      ROS_INFO_STREAM("[docking_nav] Debug log file=" << debug_log_path_);
    }
    else
    {
      ROS_WARN_STREAM("[docking_nav] Failed to open debug log file="
                      << debug_log_path_);
    }

    initCommandPublishers();

    if (!server_.initialize(private_nh_))
    {
      return false;
    }
    ROS_INFO_STREAM("[docking_nav] Initialized server with period="
                    << server_.navServerPeriod() << "s timeout="
                    << server_.opticalTimeoutSec() << "s");

    optical_sub_ = nh_.subscribe(
        topics_.optical_measurement_topic, 10,
        &DockingNavMain::opticalCallback, this);
    mode_state_sub_ = nh_.subscribe(
        topics_.mode_state_topic, 10, &DockingNavMain::modeStateCallback, this);
    heading_sub_ = nh_.subscribe(
        topics_.nav_heading_topic, 10, &DockingNavMain::headingCallback, this);
    depth_sub_ = nh_.subscribe(
        topics_.nav_depth_topic, 10, &DockingNavMain::depthCallback, this);
    dock_depth_sub_ = nh_.subscribe(
        topics_.dock_depth_topic, 10, &DockingNavMain::dockDepthCallback, this);
    pitch_sub_ = nh_.subscribe(
        topics_.nav_pitch_topic, 10, &DockingNavMain::pitchCallback, this);
    roll_sub_ = nh_.subscribe(
        topics_.nav_roll_topic, 10, &DockingNavMain::rollCallback, this);
    desired_speed_sub_ = nh_.subscribe(
        topics_.desired_speed_topic, 10,
        &DockingNavMain::desiredSpeedCallback, this);

    phase_pub_ = nh_.advertise<std_msgs::Int32>(topics_.phase_topic, 10);
    optical_xy_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>(topics_.optical_xy_topic, 10);
    optical_feedback_pub_ =
        nh_.advertise<docking_optical_msgs::OpticalFeedback>(
            topics_.optical_feedback_topic, 10);

    const ros::Duration period(server_.navServerPeriod());
    timer_ = nh_.createTimer(period, &DockingNavMain::timerCallback, this);
    ROS_INFO_STREAM("[docking_nav] Subscribed topics: optical="
                    << topics_.optical_measurement_topic
                    << " mode_state=" << topics_.mode_state_topic
                    << " heading=" << topics_.nav_heading_topic
                    << " depth=" << topics_.nav_depth_topic
                    << " dock_depth=" << topics_.dock_depth_topic
                    << " pitch=" << topics_.nav_pitch_topic
                    << " roll=" << topics_.nav_roll_topic
                    << " desired_speed=" << topics_.desired_speed_topic);
    ROS_INFO_STREAM("[docking_nav] Publishing topics: phase="
                    << topics_.phase_topic << " optical_xy="
                    << topics_.optical_xy_topic << " optical_feedback="
                    << topics_.optical_feedback_topic);
    return true;
  }

private:
  void initCommandPublishers()
  {
    bool_publishers_["STATIONING"] =
        nh_.advertise<std_msgs::Bool>(command_topics_.stationing_topic, 10);
    bool_publishers_["CONSTHEIGHT"] =
        nh_.advertise<std_msgs::Bool>(command_topics_.constheight_topic, 10);
    bool_publishers_["DOCKING_FALLING"] =
        nh_.advertise<std_msgs::Bool>(command_topics_.docking_falling_topic, 10);
    bool_publishers_["MOOS_MANUAL_OVERIDE"] =
        nh_.advertise<std_msgs::Bool>(command_topics_.manual_override_topic, 10);
    bool_publishers_["DOCKINGFAILED"] =
        nh_.advertise<std_msgs::Bool>(command_topics_.docking_failed_topic, 10);

    string_publishers_["MODE"] =
        nh_.advertise<std_msgs::String>(command_topics_.mode_topic, 10);
    string_publishers_["DOCKDEPTH_UPDATE"] = nh_.advertise<std_msgs::String>(
        command_topics_.dockdepth_update_topic, 10);
    string_publishers_["DOCKHDG_UPDATES"] = nh_.advertise<std_msgs::String>(
        command_topics_.dockhdg_updates_topic, 10);
  }

  void publishBoolCommand(const std::string &key, bool value)
  {
    auto it = bool_publishers_.find(key);
    if (it == bool_publishers_.end())
    {
      return;
    }
    std_msgs::Bool msg;
    msg.data = value;
    it->second.publish(msg);
    ROS_DEBUG_STREAM("[docking_nav] Publish bool command " << key << "="
                                                           << (value ? "true"
                                                                     : "false"));
    logDebug(std::string("[docking_nav] Publish bool command ") + key + "=" +
             (value ? "true" : "false"));
  }

  void publishStringCommand(const std::string &key, const std::string &value)
  {
    auto it = string_publishers_.find(key);
    if (it == string_publishers_.end())
    {
      return;
    }
    std_msgs::String msg;
    msg.data = value;
    it->second.publish(msg);
    ROS_INFO_STREAM("[docking_nav] Publish string command " << key << "="
                                                            << value);
  }

  void opticalCallback(
      const docking_optical_msgs::OpticalMeasurement::ConstPtr &msg)
  {
    server_.setOpticalMeasurement(*msg);
  }

  void modeStateCallback(const std_msgs::String::ConstPtr &msg)
  {
    server_.setModeFromExternal(msg->data);
  }

  void headingCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    server_.setNavHeading(msg->data);
  }

  void depthCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    server_.setNavDepth(msg->data);
  }

  void dockDepthCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    server_.setDockDepth(msg->data);
  }

  void pitchCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    server_.setNavPitch(msg->data);
  }

  void rollCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    server_.setNavRoll(msg->data);
  }

  void desiredSpeedCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    server_.setDesiredSpeed(msg->data);
  }

  void timerCallback(const ros::TimerEvent &)
  {
    const auto outputs = server_.update(ros::Time::now());

    std_msgs::Int32 phase_msg;
    phase_msg.data = outputs.phase;
    phase_pub_.publish(phase_msg);
    optical_xy_pub_.publish(outputs.optical_xy);
    optical_feedback_pub_.publish(outputs.optical_feedback);

    ROS_DEBUG_STREAM_THROTTLE(
        1.0, "[docking_nav] Outputs: phase=" << outputs.phase
                                             << " optical_xy=("
                                             << outputs.optical_xy.point.x
                                             << ","
                                             << outputs.optical_xy.point.y
                                             << ")");
    const ros::Time now = ros::Time::now();
    if ((now - last_debug_log_time_).toSec() >= 1.0)
    {
      last_debug_log_time_ = now;
      std::ostringstream stream;
      stream << "[docking_nav] Outputs: phase=" << outputs.phase
             << " optical_xy=(" << outputs.optical_xy.point.x << ","
             << outputs.optical_xy.point.y << ")";
      logDebug(stream.str());
    }
  }

  void logDebug(const std::string &message)
  {
    if (!debug_log_.is_open())
    {
      return;
    }
    debug_log_ << std::fixed << std::setprecision(3) << ros::Time::now().toSec()
               << " " << message << '\n';
    debug_log_.flush();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  DockingNavTopics topics_;
  DockingCommandTopics command_topics_;
  DockingNavServer server_;

  ros::Subscriber optical_sub_;
  ros::Subscriber heading_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber dock_depth_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber roll_sub_;
  ros::Subscriber desired_speed_sub_;
  ros::Subscriber mode_state_sub_;

  ros::Publisher phase_pub_;
  ros::Publisher optical_xy_pub_;
  ros::Publisher optical_feedback_pub_;

  ros::Timer timer_;

  std::unordered_map<std::string, ros::Publisher> bool_publishers_;
  std::unordered_map<std::string, ros::Publisher> string_publishers_;

  std::string debug_log_path_;
  std::ofstream debug_log_;
  ros::Time last_debug_log_time_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "docking_nav_main_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  DockingNavMain node(nh, private_nh);
  if (!node.initialize())
  {
    ROS_ERROR("DockingNavMain initialization failed");
    return 1;
  }

  ros::spin();
  return 0;
}
