#include <controller_manager/ControllerManager.hpp>
#include <controller_manager/ControllerBase.hpp>
#include <common_tools/StringUtils.hpp>
#include <std_msgs/Float64.h>
#include <common_msgs/Float64Stamped.h>

namespace controller_manager {

ControllerManager::ControllerManager()
{
    loader_.reset(new pluginlib::ClassLoader<ControllerBase>(
        "controller_manager", "controller_manager::ControllerBase"));
}

bool ControllerManager::init(const YAML::Node& root_cfg, ControllerNodeHandle node_handle)
{
    loadVehicleName(root_cfg);

    YAML::Node controllers_node = root_cfg["controllers"];
    if (!controllers_node || !controllers_node.IsMap()) {
        throw std::runtime_error("controllers: block missing or not a map!");
    }

    loadAndSetupControllers(controllers_node, node_handle);
    return true;
}

const std::string& ControllerManager::getVehicleName() const
{
    return vehicle_name_;
}

void ControllerManager::loadVehicleName(const YAML::Node& root_cfg)
{
    if (!root_cfg["vehicle_name"] || !root_cfg["vehicle_name"].IsScalar()) {
        throw std::runtime_error("YAML config missing required field: vehicle_name");
    }
    vehicle_name_ = root_cfg["vehicle_name"].as<std::string>();
    if (vehicle_name_.empty()) {
        throw std::runtime_error("YAML config field [vehicle_name] cannot be empty");
    }
}

void ControllerManager::loadAndSetupControllers(const YAML::Node& controllers_node, ControllerNodeHandle node_handle)
{
    for (auto it = controllers_node.begin(); it != controllers_node.end(); ++it) {
        std::string key = it->first.as<std::string>();
        YAML::Node config = it->second;

        // 自动补全 controller_name/type/variable
        if (!config["controller_name"]) config["controller_name"] = key;
        if (!config["controller_type"]) {
            auto pos = key.find('_');
            config["controller_type"] = (pos == std::string::npos) ? key : key.substr(0, pos);
        }
        if (!config["controlled_variable"]) {
            auto pos = key.find('_');
            config["controlled_variable"] = (pos == std::string::npos) ? "" : key.substr(pos + 1);
        }
        std::string controller_type = config["controller_type"].as<std::string>();
        controller_type = common_tools::StringUtils::toUpper(controller_type);
        std::string plugin_class = "controller_manager/" + controller_type + "Controller";

        try {
            auto controller = loader_->createInstance(plugin_class);
            if (!controller->init(config, node_handle)) {
                CONTROLLER_ERROR_STREAM("Init failed for controller [" << key << "]");
                continue;
            }
            ControllerBase::Context context;
            context.name = key;
            context.controller = controller;

            std::string var = controller->getControlledVariable();
            std::string ctrl_type = controller->getControllerType();
            std::string desired_topic = "/" + vehicle_name_ + "/desired_" + var;
            std::string status_topic  = "/" + vehicle_name_ + "/current_" + var;
            std::string output_topic  = "/" + vehicle_name_ + "/" + ctrl_type + "_" + var;

            context.desired_topic = desired_topic;
            context.status_topic  = status_topic;
            context.output_topic  = output_topic;

            context.desired_sub = node_handle.subscribe<std_msgs::Float64>(
                desired_topic, 1,
                [this, key](const std_msgs::Float64::ConstPtr& msg) {
                    this->handleDesiredMsg(key, msg);
                });

            context.status_sub = node_handle.subscribe<common_msgs::Float64Stamped>(
                status_topic, 1,
                [this, key](const common_msgs::Float64Stamped::ConstPtr& msg) {
                    this->handleStatusMsg(key, msg);
                });

            context.output_pub = node_handle.advertise<std_msgs::Float64>(output_topic, 1);

            controllers_[key] = context;
            CONTROLLER_INFO_STREAM("Loaded controller [" << key << "] of type [" << plugin_class << "] with topics: desired="
                                   << desired_topic << ", status=" << status_topic << ", output=" << output_topic);
        } catch (const std::exception& ex) {
            CONTROLLER_ERROR_STREAM("Plugin load failed for [" << plugin_class << "]: " << ex.what());
        }
    }
}

void ControllerManager::handleDesiredMsg(const std::string& name, const std_msgs::Float64::ConstPtr& msg)
{
    auto it = controllers_.find(name);
    if (it == controllers_.end()) return;

    ControllerBase::Context& ctx = it->second;
    ctx.desired_value = msg->data;
    ctx.desired_received = true;
}

void ControllerManager::handleStatusMsg(const std::string& name, const common_msgs::Float64Stamped::ConstPtr& msg)
{
    auto it = controllers_.find(name);
    if (it == controllers_.end()) return;
    ControllerBase::Context& ctx = it->second;

    ctx.current_value = msg->data;
    ctx.current_received = true;

    if (ctx.desired_received) {
        double dt = 0.0;
        ControllerTime curr_time = msg->header.stamp;
        if (ctx.initialized) {
            dt = (curr_time - ctx.last_time).toSec();
        }
        ctx.last_time = curr_time;
        ctx.initialized = true;

        if (dt > 0.0) {
            double output = ctx.controller->update(ctx.current_value, ctx.desired_value, dt);
            std_msgs::Float64 out_msg;
            out_msg.data = output;
            ctx.output_pub.publish(out_msg);
        }
        ctx.current_received = false;
    }
}

} // namespace controller_manager
