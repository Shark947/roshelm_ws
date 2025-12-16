#pragma once
#include <string>
#include <algorithm>
#include <cctype>

/**
 * @file EnumUtils.hpp
 * @brief 控制器常用枚举类型和与字符串互转工具
 */

namespace common_tools {

enum class ControlledVariable {
    HEADING,
    PITCH,
    ROLL,
    DEPTH,
    SPEED,
    UNKNOWN
};

namespace EnumUtils {

/// 字符串转大写
inline std::string toUpper(const std::string& str) {
    std::string ret = str;
    std::transform(ret.begin(), ret.end(), ret.begin(),
                   [](unsigned char c){ return std::toupper(c); });
    return ret;
}

/// 字符串转枚举（支持大小写，非法返回UNKNOWN）
inline ControlledVariable controlledVariableFromString(const std::string& str) {
    std::string upper = toUpper(str);
    if (upper == "HEADING") return ControlledVariable::HEADING;
    if (upper == "PITCH")   return ControlledVariable::PITCH;
    if (upper == "ROLL")    return ControlledVariable::ROLL;
    if (upper == "DEPTH")   return ControlledVariable::DEPTH;
    if (upper == "SPEED")   return ControlledVariable::SPEED;
    return ControlledVariable::UNKNOWN;
}

/// 枚举转字符串（全大写，未知返回"UNKNOWN"）
inline std::string toString(ControlledVariable var) {
    switch (var) {
        case ControlledVariable::HEADING: return "HEADING";
        case ControlledVariable::PITCH:   return "PITCH";
        case ControlledVariable::ROLL:    return "ROLL";
        case ControlledVariable::DEPTH:   return "DEPTH";
        case ControlledVariable::SPEED:   return "SPEED";
        default:                          return "UNKNOWN";
    }
}

/// 判断字符串是否为受支持的被控量
inline bool isSupportedControlledVariable(const std::string& str) {
    ControlledVariable var = controlledVariableFromString(str);
    return var != ControlledVariable::UNKNOWN;
}

} // namespace EnumUtils
} // namespace common_tools
