#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <controller_manager/RosCompat.hpp>
#include <common_tools/YamlUtils.hpp>
#include <std_msgs/Float64.h>
#include <common_msgs/Float64Stamped.h>
#include "controller_manager/ControllerTools.hpp"

namespace controller_manager {

class ControllerBase {
public:
    struct Config {
        std::string controller_name;
        std::string controller_type;
        std::string controlled_variable;
        bool is_angle = false;
        bool output_reverse = false;
        bool debug = false;
    };

    struct Context {
        std::string name;
        ControllerSharedPtr<ControllerBase> controller;

        ControllerSubscriber desired_sub;
        ControllerSubscriber status_sub;
        ControllerPublisher  output_pub;

        double desired_value = 0.0;
        bool   desired_received = false;
        double current_value = 0.0;
        bool   current_received = false;
        ControllerTime last_time;
        bool initialized = false;

        std::string desired_topic;
        std::string status_topic;
        std::string output_topic;

        double period = 0.02;
    };

    ControllerBase() = default;
    virtual ~ControllerBase() {}

    /**
     * @brief 使用YAML配置和Node初始化控制器
     */
    virtual bool init(const YAML::Node& config, ControllerNodeHandle nh) = 0;

    /**
     * @brief 控制器主逻辑
     */
    virtual double update(double current_value, double desired_value, double dt) = 0;

    // ---- 通用配置getter ----
    virtual std::string getName() const { return config_.controller_name; }
    virtual std::string getControllerType() const { return config_.controller_type; }
    virtual std::string getControlledVariable() const { return config_.controlled_variable; }
    virtual bool isOutputReverse() const { return config_.output_reverse; }
    virtual bool isAngleType() const { return config_.is_angle; }
    virtual bool isDebug() const { return config_.debug; }

protected:
    Config config_;

    bool loadCommonConfig(const YAML::Node& config) {
        try {
            config_.controller_name     = common_tools::YamlUtils::getValue<std::string>(config, "controller_name");
            config_.controller_type     = common_tools::YamlUtils::getValue<std::string>(config, "controller_type");
            config_.controlled_variable = common_tools::YamlUtils::getValue<std::string>(config, "controlled_variable");
            config_.is_angle            = common_tools::YamlUtils::getValueOrDefault<bool>(config, "is_angle", false);
            config_.output_reverse      = common_tools::YamlUtils::getValueOrDefault<bool>(config, "output_reverse", false);
            config_.debug               = common_tools::YamlUtils::getValueOrDefault<bool>(config, "debug", false);
        } catch (const std::exception& e) {
            CONTROLLER_ERROR_STREAM("[ControllerBase] Common config load failed: " << e.what());
            return false;
        }
        return true;
    }

    double computeError(double current_value, double desired_value) const {
        return ControllerTools::computeError(current_value, desired_value, config_.is_angle);
    }

    double applyOutputReverse(double output) const {
        return ControllerTools::applyOutputReverse(output, config_.output_reverse);
    }

    void debugOutput(double current, double desired, double error, double output, double dt, const std::string& tag) const {
        ControllerTools::debugOutput(current, desired, error, output, dt, config_.controller_name, tag, config_.debug);
    }
};

} // namespace controller_manager
