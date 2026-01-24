#include "bt_executor/docking_phase_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <set>
#include <sstream>

#include "bt_executor/utils.hpp"

namespace bt_executor
{

namespace
{

constexpr double kStationingInnerRatio = 0.8;
constexpr double kStationingOuterRatio = 1.2;
constexpr double kStationingMinDistance = 0.3;

inline double degToRad(double deg)
{
  return deg * kDegToRad;
}

inline double distToPoint(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

}  // namespace

bool DockingPhaseManager::initialize(ros::NodeHandle &private_nh)
{
  if (!private_nh.getParam("docking/nav_server_period", nav_server_period_))
  {
    ROS_ERROR("[bt_executor] docking/nav_server_period not set");
    return false;
  }

  private_nh.param("docking/optical_camera_delta_l", optical_camera_delta_l_, 0.0);
  private_nh.param("docking/camera_view_angle_deg", camera_view_angle_deg_, 25.0);
  private_nh.param("docking/dock_depth", dock_depth_, 0.0);
  private_nh.param("docking/dock_panel", dock_panel_, 0.0);
  private_nh.param("docking/dock_heading_deg", dock_heading_deg_, 0.0);
  private_nh.param("docking/dock_pitch_deg", dock_pitch_deg_, 0.0);
  private_nh.param("docking/dock_roll_deg", dock_roll_deg_, 0.0);
  private_nh.param("docking/depth_bias", depth_bias_, 0.0);
  private_nh.param("docking/depth_ground_bias", depth_ground_bias_, 0.0);
  private_nh.param("docking/depth_camera_bias", depth_camera_bias_, 0.0);
  private_nh.param("docking/fall_depth", fall_depth_, 0.0);
  private_nh.param("docking/dradius", dradius_, 0.0);
  private_nh.param("docking/dinheading_deg", dinheading_deg_, 0.0);
  private_nh.param("docking/distance_bias", distance_bias_, 0.3);
  private_nh.param("docking/angle_bias_deg", angle_bias_deg_, 0.5);
  private_nh.param("docking/radius1th", radius1th_, 0.0);
  private_nh.param("docking/maxtry_minute_num", max_try_minute_num_, 1);
  private_nh.param("docking/constant_depth_minute_num", constant_depth_minute_num_, 5);
  private_nh.param("docking/last_phase_float_time_ms", last_phase_float_time_ms_, 500.0);
  private_nh.param("docking/transimit_duration_sec", transmit_duration_sec_, 30);
  private_nh.param("docking/docking_max_try", docking_max_try_, 3);
  private_nh.param("docking/auto_enter_closetodocking", auto_enter_closetodocking_, false);
  private_nh.param("docking/auto_enter_duration_sec", auto_enter_duration_sec_, 2.0);
  private_nh.param("docking/align_depth_offsets", align_depth_offsets_, std::vector<double>{});

  dock_heading_deg_ = normalizeAngle360(dock_heading_deg_ + 90.0);
  imu_roll_bias_ = private_nh.param("docking/imu_roll_bias_deg", imu_roll_bias_);
  imu_pitch_bias_ = private_nh.param("docking/imu_pitch_bias_deg", imu_pitch_bias_);

  last_mode_.clear();
  phase_count_ = 0;
  docking_phase_active_ = false;

  return rebuildAlignDepths(dock_depth_);
}

void DockingPhaseManager::setOpticalMeasurement(const docking_optical_msgs::OpticalMeasurement &msg)
{
  optical_received_ = true;
  optical_measurement_ = msg;
}

void DockingPhaseManager::setDesiredSpeed(double speed)
{
  desired_speed_ = speed;
}

void DockingPhaseManager::setDockDepth(double depth)
{
  dock_depth_ = depth;
  rebuildAlignDepths(depth);
}

DockingPhaseResult DockingPhaseManager::tick(const ros::Time &stamp, const NavSnapshot &nav,
                                             const MissionSnapshot &mission)
{
  DockingPhaseResult result;

  if (std::abs(mission.dock_depth - dock_depth_) > 1e-6)
  {
    setDockDepth(mission.dock_depth);
  }

  camera_depth_ = nav.depth + depth_bias_ + depth_camera_bias_;

  handleModeChange(mission, result.commands);
  handleAutoEnterCloseToDocking(mission, nav, result);

  if (mission.mode == "DOCKING" || mission.mode == "CLOSETODOCKING")
  {
    handleDocking(stamp, nav, mission, result);
  }
  else
  {
    result.outputs.phase = phase_count_;
    updateOutputs(stamp, nav, result);
  }

  return result;
}

bool DockingPhaseManager::rebuildAlignDepths(double dock_depth)
{
  dock_depth_ = dock_depth;
  light_depth_ = dock_depth_ - dock_panel_;

  std::set<double> align_depths;
  for (double offset : align_depth_offsets_)
  {
    align_depths.insert(light_depth_ - offset);
  }

  align_depths_.clear();
  for (auto it = align_depths.begin(); it != align_depths.end(); ++it)
  {
    const double depth = *it;
    const double outer_radius =
        (light_depth_ - depth_bias_ - depth_ground_bias_ - depth) *
        std::tan(degToRad(camera_view_angle_deg_));
    double inner_radius = dradius_;
    const auto next_it = std::next(it);
    if (next_it != align_depths.end())
    {
      inner_radius = (light_depth_ - depth_bias_ - depth_ground_bias_ - *next_it) *
                     std::tan(degToRad(camera_view_angle_deg_));
    }
    align_depths_.push_back({depth, inner_radius, outer_radius});
  }

  phase_total_ = static_cast<int>(align_depths_.size()) + 1;
  ROS_INFO_STREAM("[bt_executor] Docking align depths rebuilt: count=" << align_depths_.size()
                  << " dock_depth=" << dock_depth_ << " dock_heading=" << dock_heading_deg_);
  return phase_total_ > 0;
}

void DockingPhaseManager::handleModeChange(const MissionSnapshot &mission,
                                           DockingPhaseCommands &commands)
{
  if (mission.mode == last_mode_)
  {
    return;
  }

  const std::string previous = last_mode_;
  last_mode_ = mission.mode;
  ROS_INFO_STREAM("[bt_executor] MODE change observed: " << previous << " -> " << mission.mode);

  if (mission.mode == "DOCKING")
  {
    docking_phase_active_ = true;
    phase_count_ = align_depths_.empty() ? 0 : 1;
    docking_try_count_ = 0;
    retry_last_phase_count_ = 0;
    invalid_data_count_ = 0;
    constant_depth_count_ = 0;
    continuous_docking_count_ = 0;
    auto_enter_count_ = 0;
    previous_distance_ = 0.0;

    if (!align_depths_.empty())
    {
      inner_radius_ = align_depths_[phase_count_ - 1].inner_radius;
      outer_radius_ = align_depths_[phase_count_ - 1].outer_radius;
      current_depth_ = align_depths_[phase_count_ - 1].depth;
      commands.depth_update = "depth=" + std::to_string(current_depth_);
    }
  }
  else if (mission.mode != "CLOSETODOCKING")
  {
    docking_phase_active_ = false;
    phase_count_ = 0;
    inner_radius_ = 0.0;
    outer_radius_ = 0.0;
    current_depth_ = 0.0;
  }
}

void DockingPhaseManager::handleAutoEnterCloseToDocking(const MissionSnapshot &mission,
                                                        const NavSnapshot &nav,
                                                        DockingPhaseResult &result)
{
  if (mission.mode == "DOCKING" || mission.mode == "CLOSETODOCKING")
  {
    return;
  }

  if (!auto_enter_closetodocking_ || align_depths_.empty() || !optical_received_ ||
      !optical_measurement_.valid)
  {
    auto_enter_count_ = 0;
    return;
  }

  camera_depth_ = nav.depth + depth_bias_ + depth_camera_bias_;
  const double next_x = computeNextX(nav, true);
  const double next_y = computeNextY(nav, true);
  const double distance = distToPoint(0.0, 0.0, next_x, next_y);

  if (distance <= align_depths_.front().outer_radius)
  {
    ++auto_enter_count_;
    const double freq = 1.0 / std::max(nav_server_period_, 1e-3);
    if (auto_enter_count_ >= freq * auto_enter_duration_sec_)
    {
      result.commands.mode = "CLOSETODOCKING";
      result.commands.constheight = true;
      auto_enter_count_ = 0;
    }
  }
  else
  {
    auto_enter_count_ = 0;
  }
}

void DockingPhaseManager::handleDocking(const ros::Time &stamp, const NavSnapshot &nav,
                                        const MissionSnapshot &mission, DockingPhaseResult &result)
{
  const bool optical_valid = optical_received_ && optical_measurement_.valid;

  if (optical_valid)
  {
    ++continuous_docking_count_;
    const double freq = 1.0 / std::max(nav_server_period_, 1e-3);
    if (continuous_docking_count_ > freq * transmit_duration_sec_ &&
        !docking_phase_active_ && mission.mode == "CLOSETODOCKING")
    {
      docking_phase_active_ = true;
      result.commands.mode = "DOCKING";
      result.commands.constheight = false;
    }
  }
  else
  {
    continuous_docking_count_ = 0;
  }

  camera_depth_ = nav.depth + depth_bias_ + depth_camera_bias_;

  const double next_x = computeNextX(nav, optical_valid);
  const double next_y = computeNextY(nav, optical_valid);
  distance_ = distToPoint(0.0, 0.0, next_x, next_y);

  result.outputs.phase = phase_count_;
  result.outputs.optical_valid = optical_valid;
  result.outputs.next_x = next_x;
  result.outputs.next_y = next_y;
  result.outputs.distance = distance_;

  if (phase_count_ > 0 && phase_count_ < phase_total_ && !align_depths_.empty())
  {
    current_depth_ = align_depths_[phase_count_ - 1].depth;
    result.commands.depth_update = "depth=" + std::to_string(current_depth_);

    if (optical_valid && distance_ <= outer_radius_)
    {
      invalid_data_count_ = 0;
      if (distance_ < inner_radius_ && phase_count_ < phase_total_ - 1)
      {
        ++phase_count_;
        current_depth_ = align_depths_[phase_count_ - 1].depth;
        inner_radius_ = align_depths_[phase_count_ - 1].inner_radius;
        outer_radius_ = align_depths_[phase_count_ - 1].outer_radius;

        result.commands.depth_update = "depth=" + std::to_string(current_depth_);
        result.commands.heading_update = "pwt=1";
        result.commands.stationing = true;
        if (phase_count_ == phase_total_ - 1)
        {
          result.commands.heading_update = "pwt=50";
        }
      }
      else if (phase_count_ == phase_total_ - 1)
      {
        if (distance_ < inner_radius_ * kStationingInnerRatio &&
            distance_ > kStationingMinDistance)
        {
          if (mission.stationing)
          {
            result.commands.stationing = false;
            result.commands.heading_update =
                "pwt=50,heading=" + std::to_string(dock_heading_deg_);
          }
        }
        else if (distance_ > inner_radius_ * kStationingOuterRatio)
        {
          if (!mission.stationing)
          {
            result.commands.stationing = true;
            result.commands.heading_update = "pwt=1";
          }
        }

        if ((desired_speed_ * 7.0 + distance_) < dradius_ && distance_ < previous_distance_)
        {
          if (std::abs(nav.heading - dock_heading_deg_) < dinheading_deg_)
          {
            phase_count_ = phase_total_;
            current_depth_ = dock_depth_ + fall_depth_;
            result.commands.depth_update = "depth=" + std::to_string(current_depth_);
            result.commands.heading_update =
                "pwt=200,heading=" + std::to_string(dock_heading_deg_);
            result.commands.stationing = false;
            result.commands.docking_falling = true;
          }
        }
        previous_distance_ = distance_;
      }
    }
    else if (!optical_valid || distance_ > outer_radius_)
    {
      ++invalid_data_count_;
      const double freq = 1.0 / std::max(nav_server_period_, 1e-3);
      if (invalid_data_count_ > freq * 60.0 * max_try_minute_num_)
      {
        if (phase_count_ > 1)
        {
          --phase_count_;
          invalid_data_count_ = 0;
          current_depth_ = align_depths_[phase_count_ - 1].depth;
          inner_radius_ = align_depths_[phase_count_ - 1].inner_radius;
          outer_radius_ = align_depths_[phase_count_ - 1].outer_radius;

          result.commands.depth_update = "depth=" + std::to_string(current_depth_);
          result.commands.heading_update = "pwt=1";
          result.commands.stationing = true;
          result.commands.docking_falling = false;
        }
        else if (phase_count_ == 1)
        {
          if (docking_try_count_ < docking_max_try_)
          {
            ++docking_try_count_;
            docking_phase_active_ = false;
            result.commands.mode = "CLOSETODOCKING";
            result.commands.constheight = true;
          }
          else
          {
            result.commands.docking_failed = true;
          }
        }
      }
    }
  }

  if (optical_valid)
  {
    previous_distance_ = distance_;
  }

  if (phase_count_ == phase_total_ && !align_depths_.empty())
  {
    const double depth_error = std::abs(light_depth_ - camera_depth_);
    const double pitch_error = std::abs(nav.pitch - imu_pitch_bias_ - dock_pitch_deg_);
    const double roll_error = std::abs(nav.roll - imu_roll_bias_ - dock_roll_deg_);
    const bool depth_ok = depth_error <= distance_bias_;
    const bool pitch_ok = pitch_error <= angle_bias_deg_;
    const bool roll_ok = roll_error <= angle_bias_deg_;

    if (!depth_ok || !pitch_ok || !roll_ok)
    {
      ++constant_depth_count_;
      const double freq = 1.0 / std::max(nav_server_period_, 1e-3);
      if (constant_depth_count_ > constant_depth_minute_num_ * 60.0 * freq ||
          retry_last_phase_count_ >= 5)
      {
        if (retry_last_phase_count_ >= 5 || depth_error > 1.0)
        {
          --phase_count_;
          retry_last_phase_count_ = 0;
          retry_last_phase_ = false;
          invalid_data_count_ = 0;
          current_depth_ = align_depths_[phase_count_ - 1].depth;
          inner_radius_ = align_depths_[phase_count_ - 1].inner_radius;
          outer_radius_ = align_depths_[phase_count_ - 1].outer_radius;

          result.commands.depth_update = "depth=" + std::to_string(current_depth_);
          result.commands.docking_falling = false;
        }
        else
        {
          const double freq_float = std::max(nav_server_period_, 1e-3);
          const int float_limit = static_cast<int>(last_phase_float_time_ms_ / 1000.0 / freq_float);
          if (float_iteration_ < float_limit)
          {
            ++float_iteration_;
            result.commands.manual_override = true;
          }
          else
          {
            ++retry_last_phase_count_;
            result.commands.manual_override = false;
            result.commands.docking_falling = true;
            constant_depth_count_ = 0;
            invalid_data_count_ = 0;
            float_iteration_ = 0;
          }
        }
      }
    }
    else
    {
      result.commands.docking_falling = false;
    }
  }

  result.outputs.inner_radius = inner_radius_;
  result.outputs.outer_radius = outer_radius_;
  result.outputs.current_depth = current_depth_;

  updateOutputs(stamp, nav, result);
}

void DockingPhaseManager::updateOutputs(const ros::Time &stamp, const NavSnapshot &nav,
                                        DockingPhaseResult &result)
{
  result.outputs.optical_xy.header.stamp = stamp;
  result.outputs.optical_xy.header.frame_id = "docking";
  result.outputs.optical_xy.point.x = result.outputs.next_x;
  result.outputs.optical_xy.point.y = result.outputs.next_y;
  result.outputs.optical_xy.point.z = 0.0;

  result.outputs.optical_feedback.header.stamp = stamp;
  result.outputs.optical_feedback.depth_error =
      std::max(std::abs(light_depth_ - camera_depth_), distance_bias_);
  result.outputs.optical_feedback.nav_heading_deg = nav.heading;
  result.outputs.optical_feedback.next_x = result.outputs.next_x;
  result.outputs.optical_feedback.next_y = result.outputs.next_y;

  const double current_imu_x = nav.roll;
  const double current_imu_y = nav.pitch;
  result.outputs.optical_feedback.delta_imu_x = current_imu_x - last_imu_x_;
  result.outputs.optical_feedback.delta_imu_y = current_imu_y - last_imu_y_;
  last_imu_x_ = current_imu_x;
  last_imu_y_ = current_imu_y;
}

double DockingPhaseManager::computeNextX(const NavSnapshot &nav, bool optical_valid) const
{
  if (!optical_valid)
  {
    return optical_measurement_.fallback_x;
  }
  const double heading_rad = degToRad(nav.heading);
  const double dx = (light_depth_ - camera_depth_) * std::tan(degToRad(optical_measurement_.theta_x_deg));
  const double dy = (light_depth_ - camera_depth_) * std::tan(degToRad(optical_measurement_.theta_y_deg));
  const double df_dx = dx * std::sin(heading_rad) - dy * std::cos(heading_rad) -
                       optical_camera_delta_l_ * std::sin(heading_rad);
  const double df_dy = -dy * std::sin(heading_rad) - dx * std::cos(heading_rad) +
                       optical_camera_delta_l_ * std::cos(heading_rad);
  return df_dy;
}

double DockingPhaseManager::computeNextY(const NavSnapshot &nav, bool optical_valid) const
{
  if (!optical_valid)
  {
    return optical_measurement_.fallback_y;
  }
  const double heading_rad = degToRad(nav.heading);
  const double dx = (light_depth_ - camera_depth_) * std::tan(degToRad(optical_measurement_.theta_x_deg));
  const double dy = (light_depth_ - camera_depth_) * std::tan(degToRad(optical_measurement_.theta_y_deg));
  const double df_dx = dx * std::sin(heading_rad) - dy * std::cos(heading_rad) -
                       optical_camera_delta_l_ * std::sin(heading_rad);
  const double df_dy = -dy * std::sin(heading_rad) - dx * std::cos(heading_rad) +
                       optical_camera_delta_l_ * std::cos(heading_rad);
  return df_dx;
}

}  // namespace bt_executor
