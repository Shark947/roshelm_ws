#include "bt_executor/utils.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

#include <VarDataPair.h>

namespace bt_executor
{

namespace
{

std::string trimCopy(const std::string &value)
{
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  if (begin >= end)
  {
    return std::string{};
  }
  return std::string(begin, end);
}

bool parseDoubleToken(const std::string &token, double &value)
{
  std::stringstream stream(token);
  stream >> value;
  return !(stream.fail() || !stream.eof());
}

}  // namespace

std::unordered_map<std::string, std::string> parseKeyValueList(const std::string &input)
{
  std::unordered_map<std::string, std::string> result;
  std::stringstream stream(input);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    const auto equal_pos = item.find('=');
    if (equal_pos == std::string::npos)
    {
      continue;
    }
    const std::string key = trimCopy(item.substr(0, equal_pos));
    const std::string value = trimCopy(item.substr(equal_pos + 1));
    if (!key.empty())
    {
      result[key] = value;
    }
  }
  return result;
}

std::vector<std::pair<double, double>> parseWaypoints(const std::string &input)
{
  std::vector<std::pair<double, double>> waypoints;
  std::stringstream stream(input);
  std::string item;
  while (std::getline(stream, item, ';'))
  {
    std::stringstream pair_stream(item);
    std::string xs;
    std::string ys;
    if (!std::getline(pair_stream, xs, ',') || !std::getline(pair_stream, ys))
    {
      continue;
    }
    double x = 0.0;
    double y = 0.0;
    if (parseDoubleToken(trimCopy(xs), x) && parseDoubleToken(trimCopy(ys), y))
    {
      waypoints.emplace_back(x, y);
    }
  }
  return waypoints;
}

std::optional<double> parseDouble(const std::unordered_map<std::string, std::string> &data,
                                  const std::string &key)
{
  const auto it = data.find(key);
  if (it == data.end())
  {
    return std::nullopt;
  }
  double value = 0.0;
  if (!parseDoubleToken(it->second, value))
  {
    return std::nullopt;
  }
  return value;
}

std::optional<double> parseDockDepthUpdate(const std::string &input)
{
  const auto data = parseKeyValueList(input);
  return parseDouble(data, "depth");
}

bool parseVarDataPairBool(const ::VarDataPair &pair, bool &value)
{
  if (!pair.is_string())
  {
    value = std::abs(pair.get_ddata()) > 1e-6;
    return true;
  }

  std::string text = pair.get_sdata();
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });

  if (text == "true" || text == "1")
  {
    value = true;
    return true;
  }
  if (text == "false" || text == "0")
  {
    value = false;
    return true;
  }
  return false;
}

}  // namespace bt_executor
