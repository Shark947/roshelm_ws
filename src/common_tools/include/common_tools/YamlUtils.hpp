#pragma once
#include <string>
#include <map>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <stdexcept>

/**
 * @file YamlUtils.hpp
 * @brief YAML配置文件读取、类型安全键值操作等通用工具类，header-only实现
 */

namespace common_tools {

class YamlUtils {
public:
    /**
     * @brief 加载YAML文件为YAML::Node，若文件不存在或解析失败则抛出异常
     */
    static YAML::Node loadFile(const std::string& yaml_path) {
        try {
            return YAML::LoadFile(yaml_path);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load YAML file: " + yaml_path + ", error: " + e.what());
        }
    }

    /**
     * @brief 获取key对应的值（类型安全，缺失时抛出异常）
     */
    template<typename T>
    static T getValue(const YAML::Node& node, const std::string& key) {
        if (!node[key] || node[key].IsNull()) {
            throw std::runtime_error("YAML missing required key: " + key);
        }
        try {
            return node[key].as<T>();
        } catch (const std::exception& e) {
            throw std::runtime_error("YAML type error for key: " + key + ", error: " + e.what());
        }
    }

    /**
     * @brief 获取key对应的值，若缺失则返回默认值
     */
    template<typename T>
    static T getValueOrDefault(const YAML::Node& node, const std::string& key, const T& default_value) {
        if (!node[key] || node[key].IsNull()) {
            return default_value;
        }
        try {
            return node[key].as<T>();
        } catch (const std::exception&) {
            return default_value;
        }
    }

    /**
     * @brief 判断key是否存在且非空
     */
    static bool hasKey(const YAML::Node& node, const std::string& key) {
        return node[key] && !node[key].IsNull();
    }

    /**
     * @brief 将YAML节点的所有一级子节点转换为map
     */
    static std::map<std::string, YAML::Node> toMap(const YAML::Node& node) {
        std::map<std::string, YAML::Node> result;
        for (auto it = node.begin(); it != node.end(); ++it) {
            result[it->first.as<std::string>()] = it->second;
        }
        return result;
    }

    /**
     * @brief 将YAML节点序列化为字符串，调试用
     */
    static std::string toString(const YAML::Node& node) {
        std::stringstream ss;
        ss << node;
        return ss.str();
    }
};

} // namespace common_tools
