#pragma once

#include <mutex>
#include <string>

namespace bt_executor
{

struct MissionSnapshot
{
  std::string mode{"LIFT"};
  bool deploy{false};
  bool should_return{false};
  bool stationing{true};
  bool constheight{true};
  bool docking_falling{false};
  bool manual_override{false};
  bool docking_failed{false};

  double dock_heading_deg{0.0};
  double dock_depth{0.0};
  double dock_pitch_deg{0.0};
  double dock_roll_deg{0.0};

  int phase{0};
  int phase_total{0};
  bool docking_phase_active{false};
  bool optical_valid{false};
  double distance{0.0};
  double inner_radius{0.0};
  double outer_radius{0.0};
  double current_depth{0.0};
};

class MissionState
{
public:
  explicit MissionState(std::string initial_mode)
  {
    snapshot_.mode = std::move(initial_mode);
  }

  MissionSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  void setMode(const std::string &mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.mode = mode;
  }

  void setDeploy(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.deploy = value;
  }

  void setReturn(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.should_return = value;
  }

  void setStationing(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.stationing = value;
  }

  void setConstHeight(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.constheight = value;
  }

  void setDockingFalling(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.docking_falling = value;
  }

  void setManualOverride(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.manual_override = value;
  }

  void setDockingFailed(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.docking_failed = value;
  }

  void setDockingTargets(double heading_deg, double depth, double pitch_deg,
                         double roll_deg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.dock_heading_deg = heading_deg;
    snapshot_.dock_depth = depth;
    snapshot_.dock_pitch_deg = pitch_deg;
    snapshot_.dock_roll_deg = roll_deg;
  }

  void setDockDepth(double depth)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.dock_depth = depth;
  }

  void setDockHeading(double heading_deg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.dock_heading_deg = heading_deg;
  }

  void updateDockingTelemetry(int phase, int phase_total, double current_depth,
                              double inner_radius, double outer_radius,
                              bool docking_phase_active, bool optical_valid,
                              double distance)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.phase = phase;
    snapshot_.phase_total = phase_total;
    snapshot_.current_depth = current_depth;
    snapshot_.inner_radius = inner_radius;
    snapshot_.outer_radius = outer_radius;
    snapshot_.docking_phase_active = docking_phase_active;
    snapshot_.optical_valid = optical_valid;
    snapshot_.distance = distance;
  }

private:
  mutable std::mutex mutex_;
  MissionSnapshot snapshot_{};
};

}  // namespace bt_executor
