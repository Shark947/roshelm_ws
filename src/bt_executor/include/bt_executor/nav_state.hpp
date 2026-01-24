#pragma once

#include <mutex>

namespace bt_executor
{

struct NavSnapshot
{
  double x{0.0};
  double y{0.0};
  double depth{0.0};
  double heading{0.0};
  double speed{0.0};
  double pitch{0.0};
  double roll{0.0};
};

class NavState
{
public:
  void setX(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.x = value;
  }

  void setY(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.y = value;
  }

  void setDepth(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.depth = value;
  }

  void setHeading(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.heading = value;
  }

  void setSpeed(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.speed = value;
  }

  void setPitch(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.pitch = value;
  }

  void setRoll(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.roll = value;
  }

  NavSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

private:
  mutable std::mutex mutex_;
  NavSnapshot state_{};
};

}  // namespace bt_executor
