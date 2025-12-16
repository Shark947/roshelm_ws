#pragma once

/**
 * @file AngleUtils.hpp
 * @brief 角度和弧度常用数学工具函数，全部为命名空间下的函数
 */

namespace common_tools {
namespace AngleUtils {

/// 将角度归一化到 [-180, 180)
inline double normalize180(double angle_deg) {
    while (angle_deg >= 180.0) angle_deg -= 360.0;
    while (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

/// 将角度归一化到 [0, 360)
inline double normalize360(double angle_deg) {
    while (angle_deg >= 360.0) angle_deg -= 360.0;
    while (angle_deg < 0.0) angle_deg += 360.0;
    return angle_deg;
}

/// 将弧度归一化到 [-PI, PI)
inline double normalizeRadPI(double angle_rad) {
    const double PI = 3.14159265358979323846;
    while (angle_rad >= PI) angle_rad -= 2 * PI;
    while (angle_rad < -PI) angle_rad += 2 * PI;
    return angle_rad;
}

/// 度转弧度
inline double deg2rad(double angle_deg) {
    return angle_deg * 3.14159265358979323846 / 180.0;
}

/// 弧度转度
inline double rad2deg(double angle_rad) {
    return angle_rad * 180.0 / 3.14159265358979323846;
}

} // namespace AngleUtils
} // namespace common_tools
