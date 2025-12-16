#ifndef VARIABLE_EXTRACTOR_VARIABLE_EXTRACTOR_INTERFACE_HPP
#define VARIABLE_EXTRACTOR_VARIABLE_EXTRACTOR_INTERFACE_HPP

#include <ros/ros.h>
#include <string>
#include <map>

namespace variable_extractor {

/**
 * @brief VariableExtractorInterface：抽象接口，用于“根据变量名自动订阅并提取数值”。
 *
 * 派生类必须实现：
 *   - void setVehicleName(const std::string& vn);
 *   - void subscribe(const std::string& var_name,
 *                    ros::NodeHandle& nh_parent,
 *                    std::map<std::string, double>& var_map,
 *                    bool debug);
 */
class VariableExtractorInterface {
public:
    virtual ~VariableExtractorInterface() = default;

    /// 设置车辆名称（用于拼接 Topic 前缀）
    virtual void setVehicleName(const std::string& vn) = 0;

    /**
     * @brief 为 var_name 创建订阅，并在收到新数据时更新 var_map[var_name]。
     * @param var_name  变量名称 (例如 "DEPTH","SPEED","HEADING" 等，大小写不限)
     * @param nh_parent 用于创建 Subscriber/Publisher 的 NodeHandle（通常是全局命名空间）
     * @param var_map   外部传入的 map，用来存放提取到的数值： var_map[var_name] = value
     * @param debug     如果为 true，则打印调试信息
     */
    virtual void subscribe(const std::string& var_name,
                           ros::NodeHandle& nh_parent,
                           std::map<std::string, double>& var_map,
                           bool debug) = 0;

     // 新增：返回本插件支持的所有变量名
    virtual std::vector<std::string> supportedVariables() const = 0;
};

}  // namespace variable_extractor

#endif  // VARIABLE_EXTRACTOR_VARIABLE_EXTRACTOR_INTERFACE_HPP
