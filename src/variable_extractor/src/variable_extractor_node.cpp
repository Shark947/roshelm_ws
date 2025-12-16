#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "variable_extractor/VariableExtractorInterface.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "variable_extractor_node");
    ros::NodeHandle nh("~");

    std::string vehicle_name;
    if (!nh.getParam("vehicle_name", vehicle_name) || vehicle_name.empty()) {
        ROS_ERROR_STREAM("[variable_extractor_node] vehicle_name param missing or empty!");
        return 1;
    } else {
        ROS_INFO_STREAM("[variable_extractor_node] vehicle_name param = [" << vehicle_name << "]");
    }
    bool debug = false;
    nh.param<bool>("debug", debug, false);

    // 2. 自动发现所有插件
    pluginlib::ClassLoader<variable_extractor::VariableExtractorInterface> loader(
        "variable_extractor",
        "variable_extractor::VariableExtractorInterface"
    );
    auto plugin_types = loader.getDeclaredClasses();

    ROS_INFO_STREAM("Discovered VariableExtractor plugins:");
    for (const auto& type : plugin_types) {
        ROS_INFO_STREAM(" - " << type);
    }

    // 3. 加载所有插件
    std::vector<boost::shared_ptr<variable_extractor::VariableExtractorInterface>> plugins;
    for (const auto& type : plugin_types) {
        try {
            auto plugin = loader.createInstance(type);
            plugin->setVehicleName(vehicle_name);
            plugins.push_back(plugin);
            ROS_INFO_STREAM("[variable_extractor_node] Loaded plugin: " << type);

            // 4. 获取支持的变量并自动订阅
            auto vars = plugin->supportedVariables();
            static std::map<std::string, double> dummy_map; // 改为 static
            for (const auto& var : vars) {
                plugin->subscribe(var, nh, dummy_map, debug);
                ROS_INFO_STREAM("  Subscribed variable: " << var);
            }
        } catch (const pluginlib::PluginlibException& e) {
            ROS_ERROR_STREAM("Failed to load plugin [" << type << "]: " << e.what());
        }
    }

    ROS_INFO_STREAM("variable_extractor_node started for vehicle: " << vehicle_name);
    ros::spin();
    return 0;
}
