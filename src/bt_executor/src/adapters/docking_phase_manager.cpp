#include "bt_executor/adapters/docking_phase_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <set>

#include "bt_executor/utils/utils.hpp"

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

bool DockingPhaseManager::initialize(const DockingPhaseManagerConfig &config)
{
  nav_server_period_ = std::max(config.nav_server_period, 1e-3);
  dock_heading_deg_ = normalizeAngle360(config.dock_heading_deg + 90.0);
  dock_pitch_deg_ = config.dock_pitch_deg;
  dock_roll_deg_ = config.dock_roll_deg;
  imu_roll_bias_ = config.imu_roll_bias_deg;
  imu_pitch_bias_ = config.imu_pitch_bias_deg;
  depth_bias_ = config.depth_bias;
  distance_bias_ = config.distance_bias;
  angle_bias_deg_ = config.angle_bias_deg;
  fall_depth_ = config.fall_depth;
  camera_view_angle_deg_ = config.camera_view_angle_deg;
  max_try_minute_num_ = config.max_try_minute_num;
  constant_depth_minute_num_ = config.constant_depth_minute_num;
  depth_ground_bias_ = config.depth_ground_bias;
  depth_camera_bias_ = config.depth_camera_bias;
  dock_panel_ = config.dock_panel;
  last_phase_float_time_ms_ = config.last_phase_float_time_ms;
  transmit_duration_sec_ = config.transmit_duration_sec;
  docking_max_try_ = config.docking_max_try;
  optical_camera_delta_l_ = config.optical_camera_delta_l;
  align_depth_offsets_ = config.align_depth_offsets;

  dradius_ = config.dradius;
  dinheading_deg_ = config.dinheading_deg;
  radius1th_ = config.radius1th;

  auto_enter_closetodocking_ = config.auto_enter_closetodocking;
  auto_enter_duration_sec_ = config.auto_enter_duration_sec;

  last_mode_.clear();
  phase_count_ = 0;
  docking_phase_active_ = false;
  optical_received_ = false;

  return rebuildAlignDepths(config.dock_depth);
}

void DockingPhaseManager::setOpticalMeasurement(const OpticalMeasurementData &measurement)
{
  optical_received_ = true;
  optical_measurement_ = measurement;
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

DockingPhaseResult DockingPhaseManager::tick(const NavSnapshot &nav, const MissionSnapshot &mission)
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
    handleDocking(nav, mission, result);
  }
  else
  {
    result.outputs.phase = phase_count_;
    updateOutputs(nav, result);
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
  return phase_total_ > 0;
}

void DockingPhaseManager::handleModeChange(const MissionSnapshot &mission, DockingPhaseCommands &commands)
{
  if (mission.mode == last_mode_)
  {
    return;
  }

  last_mode_ = mission.mode;

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
    const double freq = 1.0 / nav_server_period_;
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

void DockingPhaseManager::handleDocking(const NavSnapshot &nav, const MissionSnapshot &mission,
                                        DockingPhaseResult &result)
{
  const bool optical_valid = optical_received_ && optical_measurement_.valid;

  if (optical_valid)
  {
    ++continuous_docking_count_;
    const double freq = 1.0 / nav_server_period_;
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
        if (distance_ < inner_radius_ * kStationingInnerRatio && distance_ > kStationingMinDistance)
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
      const double freq = 1.0 / nav_server_period_;
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
      const double freq = 1.0 / nav_server_period_;
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
          const double freq_float = nav_server_period_;
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

  updateOutputs(nav, result);
}

void DockingPhaseManager::updateOutputs(const NavSnapshot &nav, DockingPhaseResult &result)
{
  result.outputs.nav_heading_deg = nav.heading;
  result.outputs.depth_error = std::max(std::abs(light_depth_ - camera_depth_), distance_bias_);

  const double current_imu_x = nav.roll;
  const double current_imu_y = nav.pitch;
  result.outputs.delta_imu_x = current_imu_x - last_imu_x_;
  result.outputs.delta_imu_y = current_imu_y - last_imu_y_;
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
