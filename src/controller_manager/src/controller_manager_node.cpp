#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <controller_manager/ControllerManager.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_manager_node");
    ros::NodeHandle nh("~");

    // --- 1. 加载YAML配置 ---
    std::string config_path;
    nh.param<std::string>("controllers_config", config_path, "config/controllers.yaml");

    YAML::Node root_cfg;
    try {
        root_cfg = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to load YAML config: " << e.what());
        return 1;
    }

    YAML::Node controllers_node = root_cfg["controllers"];
    if (!controllers_node || !controllers_node.IsMap()) {
        ROS_ERROR_STREAM("controllers: block missing or not a map!");
        return 1;
    }

    // --- 2. 初始化控制器管理器（加载插件、拼接话题并注册回调） ---
    controller_manager::ControllerManager manager;
    try {
        manager.init(root_cfg, nh);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to initialize controller manager: " << e.what());
        return 1;
    }

    // --- 4. spin循环 ---
    ros::spin();
    return 0;
}
