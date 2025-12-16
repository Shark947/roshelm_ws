#pragma once

#include <string>
#include <common_tools/AngleUtils.hpp>
#include <controller_manager/RosCompat.hpp>

namespace controller_manager {

class ControllerTools {
public:
    ControllerTools() = delete;

    static double computeError(double current_value, double desired_value, bool is_angle) {
        if (is_angle) {
            double c = common_tools::AngleUtils::normalize180(current_value);
            double d = common_tools::AngleUtils::normalize180(desired_value);
            return common_tools::AngleUtils::normalize180(c - d);
        }
        return current_value - desired_value;
    }

    static double applyOutputReverse(double output, bool output_reverse) {
        return output_reverse ? -output : output;
    }

    static void debugOutput(double current, double desired, double error, double output, double dt,
                            const std::string& controller_name, const std::string& tag, bool debug) {
        if (!debug) return;
        CONTROLLER_INFO_STREAM_THROTTLE(1.0,
            "[" << tag << "-" << controller_name << "] desired=" << desired
            << ", current=" << current
            << ", error="   << error
            << ", output="  << output
            << ", dt=" << dt);
    }
};

} // namespace controller_manager
