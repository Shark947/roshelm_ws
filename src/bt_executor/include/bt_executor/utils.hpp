#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bt_executor
{

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

inline double normalizeAngle360(double angle_deg)
{
  double value = std::fmod(angle_deg, 360.0);
  if (value < 0.0)
  {
    value += 360.0;
  }
  return value;
}

inline double shortestAngleDiffDeg(double from_deg, double to_deg)
{
  double diff = normalizeAngle360(to_deg) - normalizeAngle360(from_deg);
  if (diff > 180.0)
  {
    diff -= 360.0;
  }
  else if (diff < -180.0)
  {
    diff += 360.0;
  }
  return diff;
}

inline double distance2D(double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

inline double bearingDeg(double x0, double y0, double x1, double y1)
{
  const double heading_rad = std::atan2(y1 - y0, x1 - x0);
  return normalizeAngle360(heading_rad * kRadToDeg);
}

std::unordered_map<std::string, std::string> parseKeyValueList(const std::string &input);

std::vector<std::pair<double, double>> parseWaypoints(const std::string &input);

std::optional<double> parseDouble(const std::unordered_map<std::string, std::string> &data,
                                  const std::string &key);

std::optional<double> parseDockDepthUpdate(const std::string &input);

}  // namespace bt_executor
