#pragma once
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <ostream>

/**
 * @file StringUtils.hpp
 * @brief 常用字符串处理静态工具类，全部函数为静态方法
 */

namespace common_tools {

/**
 * @class StringUtils
 * @brief 提供大小写转换、空白去除、分割与连接等字符串工具函数
 */
class StringUtils {
public:
    /// 将字符串全部转为大写
    static inline std::string toUpper(const std::string& str) {
        std::string result(str);
        std::transform(result.begin(), result.end(), result.begin(),
                       [](unsigned char c){ return std::toupper(c); });
        return result;
    }

    /// 将字符串全部转为小写
    static inline std::string toLower(const std::string& str) {
        std::string result(str);
        std::transform(result.begin(), result.end(), result.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return result;
    }

    /// 去除字符串首尾空白字符（空格、Tab、换行等）
    static inline std::string trim(const std::string& str) {
        const auto strBegin = str.find_first_not_of(" \t\n\r");
        if (strBegin == std::string::npos) return "";
        const auto strEnd = str.find_last_not_of(" \t\n\r");
        const auto strRange = strEnd - strBegin + 1;
        return str.substr(strBegin, strRange);
    }

    /// 按分隔符拆分字符串
    static inline std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, delimiter)) {
            tokens.push_back(item);
        }
        return tokens;
    }

    /// 按分隔符连接字符串数组
    static inline std::string join(const std::vector<std::string>& strs, const std::string& delimiter) {
        if (strs.empty()) return "";
        std::ostringstream oss;
        auto it = strs.begin();
        oss << *it++;
        for (; it != strs.end(); ++it) {
            oss << delimiter << *it;
        }
        return oss.str();
    }
};

} // namespace common_tools
