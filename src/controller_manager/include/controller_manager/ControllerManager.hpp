#pragma once

#include <string>
#include <map>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <controller_manager/RosCompat.hpp>
#include <controller_manager/ControllerBase.hpp>
#include <std_msgs/Float64.h>
#include <common_msgs/Float64Stamped.h>
#include <pluginlib/class_loader.hpp>

namespace controller_manager {

class ControllerManager {
public:
    ControllerManager();

    bool init(const YAML::Node& root_cfg, ControllerNodeHandle node_handle);
    const std::string& getVehicleName() const;

private:
    std::unique_ptr<pluginlib::ClassLoader<ControllerBase>> loader_;
    std::string vehicle_name_;
    std::map<std::string, ControllerBase::Context> controllers_;

    void loadVehicleName(const YAML::Node& root_cfg);
    void loadAndSetupControllers(const YAML::Node& controllers_node, ControllerNodeHandle node_handle);

    void handleDesiredMsg(const std::string& name, const std_msgs::Float64::ConstPtr& msg);
    void handleStatusMsg(const std::string& name, const common_msgs::Float64Stamped::ConstPtr& msg);
};

} // namespace controller_manager
