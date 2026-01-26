#pragma once

#include <map>
#include <sstream>
#include <string>

namespace ros_behavior_tree
{

inline std::string trimCopy(const std::string &value)
{
  const std::string whitespace = " \t\n\r";
  const auto start = value.find_first_not_of(whitespace);
  if (start == std::string::npos)
    return std::string();
  const auto end = value.find_last_not_of(whitespace);
  return value.substr(start, end - start + 1);
}

inline std::map<std::string, std::string> parseParams(
    const std::string &params)
{
  std::map<std::string, std::string> result;
  std::stringstream stream(params);
  std::string token;
  while (std::getline(stream, token, ';'))
  {
    const std::string trimmed = trimCopy(token);
    if (trimmed.empty())
      continue;
    const auto equals_pos = trimmed.find('=');
    if (equals_pos == std::string::npos)
      continue;
    const std::string key = trimCopy(trimmed.substr(0, equals_pos));
    const std::string value = trimCopy(trimmed.substr(equals_pos + 1));
    if (!key.empty())
      result[key] = value;
  }
  return result;
}

}  // namespace ros_behavior_tree
