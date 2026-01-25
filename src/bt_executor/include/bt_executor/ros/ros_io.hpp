#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <common_msgs/Float64Stamped.h>
#include <docking_optical_msgs/OpticalMeasurement.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <VarDataPair.h>

#include "bt_executor/bt_context.hpp"
#include "bt_executor/adapters/docking_phase_manager.hpp"
#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/ros/command_publisher.hpp"
#include "bt_executor/ros/docking_outputs_publisher.hpp"

namespace bt_executor
{

struct TopicConfig
{
  std::string nav_x_topic{"/auh/NAV_X"};
  std::string nav_y_topic{"/auh/NAV_Y"};
  std::string nav_depth_topic{"/auh/NAV_DEPTH"};
  std::string nav_heading_topic{"/auh/NAV_HEADING"};
  std::string nav_speed_topic{"/auh/NAV_SPEED"};
  std::string nav_pitch_topic{"/auh/NAV_PITCH"};
  std::string nav_roll_topic{"/auh/NAV_ROLL"};

  std::string deploy_topic{"/auh/DEPLOY"};
  std::string return_topic{"/auh/RETURN"};

  std::string mode_topic{"/docking/mode"};
  std::string mode_state_topic{"/docking/mode_state"};

  std::string stationing_topic{"/docking/stationing"};
  std::string constheight_topic{"/docking/constheight"};
  std::string docking_falling_topic{"/docking/docking_falling"};
  std::string manual_override_topic{"/docking/manual_override"};
  std::string docking_failed_topic{"/docking/docking_failed"};
  std::string dockdepth_update_topic{"/docking/dockdepth_update"};
  std::string dockhdg_updates_topic{"/docking/dockhdg_updates"};

  std::string optical_measurement_topic{"/docking/optical_measurement"};
  std::string dock_depth_topic{"/docking/dock_depth"};

  std::string phase_topic{"/docking/phase"};
  std::string optical_xy_topic{"/docking/optical_xy"};
  std::string optical_feedback_topic{"/docking/optical_feedback"};

  std::string desired_heading_topic{"/auh/desired_heading"};
  std::string desired_speed_topic{"/auh/desired_speed"};
  std::string desired_depth_topic{"/auh/desired_depth"};
};

struct CommandTopics
{
  std::unordered_map<std::string, std::string> bool_topics{
      {"STATIONING", "/docking/stationing"},
      {"CONSTHEIGHT", "/docking/constheight"},
      {"DOCKING_FALLING", "/docking/docking_falling"},
      {"MOOS_MANUAL_OVERIDE", "/docking/manual_override"},
      {"DOCKINGFAILED", "/docking/docking_failed"}};

  std::unordered_map<std::string, std::string> string_topics{
      {"MODE", "/docking/mode"},
      {"DOCKDEPTH_UPDATE", "/docking/dockdepth_update"},
      {"DOCKHDG_UPDATES", "/docking/dockhdg_updates"}};
};

struct CommandState
{
  double heading_deg{0.0};
  double speed{0.0};
  double depth{0.0};
  bool initialized{false};
};

class RosIO
{
public:
  RosIO() = default;

  bool init(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
            const std::shared_ptr<BTContext> &ctx);

  double now() const;

  void publishCommands(const HelmDecision &decision, double stamp);
  void publishModeState(const std::string &mode);

  void applyBehaviorMessages(const std::vector<VarDataPair> &messages, double stamp);
  void applyDockingCommands(const DockingPhaseCommands &commands, double stamp);

  void publishDockingOutputs(const DockingPhaseOutputs &outputs, double stamp);

  double lastCommandSpeed(double fallback_speed) const;

private:
  bool loadParameters(ros::NodeHandle &private_nh);
  void setupSubscribers();
  void setupPublishers();

  void navXCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void navYCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void navDepthCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void navHeadingCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void navSpeedCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void navPitchCallback(const common_msgs::Float64Stamped::ConstPtr &msg);
  void navRollCallback(const common_msgs::Float64Stamped::ConstPtr &msg);

  void deployCallback(const std_msgs::Bool::ConstPtr &msg);
  void returnCallback(const std_msgs::Bool::ConstPtr &msg);

  void modeCallback(const std_msgs::String::ConstPtr &msg);
  void stationingCallback(const std_msgs::Bool::ConstPtr &msg);
  void constHeightCallback(const std_msgs::Bool::ConstPtr &msg);
  void dockingFallingCallback(const std_msgs::Bool::ConstPtr &msg);
  void manualOverrideCallback(const std_msgs::Bool::ConstPtr &msg);
  void dockingFailedCallback(const std_msgs::Bool::ConstPtr &msg);
  void dockDepthUpdateCallback(const std_msgs::String::ConstPtr &msg);
  void dockHeadingUpdateCallback(const std_msgs::String::ConstPtr &msg);
  void opticalCallback(const docking_optical_msgs::OpticalMeasurement::ConstPtr &msg);
  void dockDepthCallback(const common_msgs::Float64Stamped::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  TopicConfig topics_{};
  CommandTopics command_topics_{};

  std::shared_ptr<BTContext> ctx_;

  std::shared_ptr<DockingOutputsPublisher> outputs_publisher_;
  std::shared_ptr<CommandPublisher> command_publisher_;

  ros::Subscriber nav_x_sub_;
  ros::Subscriber nav_y_sub_;
  ros::Subscriber nav_depth_sub_;
  ros::Subscriber nav_heading_sub_;
  ros::Subscriber nav_speed_sub_;
  ros::Subscriber nav_pitch_sub_;
  ros::Subscriber nav_roll_sub_;

  ros::Subscriber deploy_sub_;
  ros::Subscriber return_sub_;

  ros::Subscriber mode_sub_;
  ros::Subscriber stationing_sub_;
  ros::Subscriber constheight_sub_;
  ros::Subscriber docking_falling_sub_;
  ros::Subscriber manual_override_sub_;
  ros::Subscriber docking_failed_sub_;
  ros::Subscriber dockdepth_update_sub_;
  ros::Subscriber dockhdg_updates_sub_;
  ros::Subscriber optical_sub_;
  ros::Subscriber dock_depth_sub_;

  ros::Publisher desired_heading_pub_;
  ros::Publisher desired_speed_pub_;
  ros::Publisher desired_depth_pub_;
  ros::Publisher mode_state_pub_;

  CommandState last_command_{};
  std::string last_mode_published_;
};

}  // namespace bt_executor
