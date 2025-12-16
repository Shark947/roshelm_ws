#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <stdexcept>

/**
 * @file ParamChecker.hpp
 * @brief 通用参数检查工具，仅适用于 YAML 配置检查。无 ROS 依赖。
 */

namespace common_tools {

namespace ParamChecker {

/**
 * @brief 检查配置节点中是否包含所有必需的键（required_keys）
 * @param config YAML节点
 * @param required_keys 必需键名列表
 * @throws std::runtime_error 若有缺失
 */
inline void checkYamlKeys(const YAML::Node& config, const std::vector<std::string>& required_keys) {
    for (const auto& key : required_keys) {
        if (!config[key]) {
            throw std::runtime_error("Missing required YAML key: " + key);
        }
    }
}

// 可后续扩展其它 YAML 检查 inline 函数

} // namespace ParamChecker

} // namespace common_tools
