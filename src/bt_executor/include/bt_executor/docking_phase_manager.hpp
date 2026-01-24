#pragma once

#include <optional>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <docking_optical_msgs/OpticalFeedback.h>
#include <docking_optical_msgs/OpticalMeasurement.h>
#include <geometry_msgs/PointStamped.h>

#include "bt_executor/mission_state.hpp"
#include "bt_executor/nav_state.hpp"

namespace bt_executor
{

struct DockingPhaseOutputs
{
  int phase{0};
  geometry_msgs::PointStamped optical_xy;
  docking_optical_msgs::OpticalFeedback optical_feedback;

  bool optical_valid{false};
  double next_x{0.0};
  double next_y{0.0};
  double distance{0.0};
  double inner_radius{0.0};
  double outer_radius{0.0};
  double current_depth{0.0};
};

struct DockingPhaseCommands
{
  std::optional<std::string> mode;
  std::optional<std::string> depth_update;
  std::optional<std::string> heading_update;

  std::optional<bool> stationing;
  std::optional<bool> constheight;
  std::optional<bool> docking_falling;
  std::optional<bool> manual_override;
  std::optional<bool> docking_failed;
};

struct DockingPhaseResult
{
  DockingPhaseOutputs outputs;
  DockingPhaseCommands commands;
};

class DockingPhaseManager
{
public:
  DockingPhaseManager() = default;

  bool initialize(ros::NodeHandle &private_nh);

  void setOpticalMeasurement(const docking_optical_msgs::OpticalMeasurement &msg);
  void setDesiredSpeed(double speed);
  void setDockDepth(double depth);

  DockingPhaseResult tick(const ros::Time &stamp, const NavSnapshot &nav,
                          const MissionSnapshot &mission);

  double dockHeadingDeg() const { return dock_heading_deg_; }
  double dockDepth() const { return dock_depth_; }
  double dockPitchDeg() const { return dock_pitch_deg_; }
  double dockRollDeg() const { return dock_roll_deg_; }
  int phaseTotal() const { return phase_total_; }

private:
  struct AlignDepth
  {
    double depth{0.0};
    double inner_radius{0.0};
    double outer_radius{0.0};
  };

  bool rebuildAlignDepths(double dock_depth);
  void handleModeChange(const MissionSnapshot &mission, DockingPhaseCommands &commands);
  void handleAutoEnterCloseToDocking(const MissionSnapshot &mission,
                                     const NavSnapshot &nav,
                                     DockingPhaseResult &result);
  void handleDocking(const ros::Time &stamp, const NavSnapshot &nav,
                     const MissionSnapshot &mission, DockingPhaseResult &result);

  void updateOutputs(const ros::Time &stamp, const NavSnapshot &nav,
                     DockingPhaseResult &result);

  double computeNextX(const NavSnapshot &nav, bool optical_valid) const;
  double computeNextY(const NavSnapshot &nav, bool optical_valid) const;

  double nav_server_period_{0.1};
  double dock_heading_deg_{0.0};
  double dock_pitch_deg_{0.0};
  double dock_roll_deg_{0.0};
  double imu_roll_bias_{0.0};
  double imu_pitch_bias_{0.0};
  double depth_bias_{0.0};
  double distance_bias_{0.3};
  double angle_bias_deg_{0.5};
  double fall_depth_{0.0};
  double camera_view_angle_deg_{25.0};
  int max_try_minute_num_{1};
  int constant_depth_minute_num_{5};
  double depth_ground_bias_{0.0};
  double depth_camera_bias_{0.0};
  double dock_panel_{0.0};
  double last_phase_float_time_ms_{500.0};
  int transmit_duration_sec_{30};
  int docking_max_try_{3};
  double optical_camera_delta_l_{0.0};

  std::vector<double> align_depth_offsets_;
  std::vector<AlignDepth> align_depths_;
  int phase_total_{1};

  double dradius_{0.0};
  double dinheading_deg_{0.0};
  double radius1th_{0.0};

  bool auto_enter_closetodocking_{false};
  double auto_enter_duration_sec_{2.0};
  int auto_enter_count_{0};

  docking_optical_msgs::OpticalMeasurement optical_measurement_{};
  bool optical_received_{false};

  double desired_speed_{0.0};
  double dock_depth_{0.0};
  double light_depth_{0.0};
  double camera_depth_{0.0};

  int phase_count_{0};
  bool docking_phase_active_{false};
  bool retry_last_phase_{false};

  double current_depth_{0.0};
  double inner_radius_{0.0};
  double outer_radius_{0.0};
  double previous_distance_{0.0};
  double distance_{0.0};

  int docking_try_count_{0};
  int retry_last_phase_count_{0};
  int float_iteration_{0};
  int invalid_data_count_{0};
  int constant_depth_count_{0};
  int continuous_docking_count_{0};

  double last_imu_x_{0.0};
  double last_imu_y_{0.0};

  std::string last_mode_;
};

}  // namespace bt_executor
