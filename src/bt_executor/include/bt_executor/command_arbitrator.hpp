#pragma once

#include <array>
#include <limits>
#include <optional>
#include <string>

#include "bt_executor/nav_state.hpp"

namespace bt_executor
{

enum class CommandDimension
{
  Heading = 0,
  Speed = 1,
  Depth = 2
};

struct CommandRequest
{
  CommandDimension dimension{CommandDimension::Heading};
  double value{0.0};
  double weight{0.0};
  std::string source;
};

struct CommandSet
{
  double heading_deg{0.0};
  double speed{0.0};
  double depth{0.0};
};

class CommandArbitrator
{
public:
  void reset()
  {
    for (auto &entry : best_)
    {
      entry.weight = -std::numeric_limits<double>::infinity();
      entry.has_value = false;
      entry.value = 0.0;
      entry.source.clear();
    }
  }

  void submit(const CommandRequest &request)
  {
    auto &entry = best_[static_cast<std::size_t>(request.dimension)];
    if (!entry.has_value || request.weight >= entry.weight)
    {
      entry.has_value = true;
      entry.weight = request.weight;
      entry.value = request.value;
      entry.source = request.source;
    }
  }

  CommandSet finalize(const NavSnapshot &nav, const CommandSet &last_command) const
  {
    CommandSet result = last_command;

    applyFallback(CommandDimension::Heading, nav.heading, result.heading_deg);
    applyFallback(CommandDimension::Speed, 0.0, result.speed);
    applyFallback(CommandDimension::Depth, nav.depth, result.depth);

    return result;
  }

private:
  struct BestEntry
  {
    double weight{-std::numeric_limits<double>::infinity()};
    double value{0.0};
    bool has_value{false};
    std::string source;
  };

  void applyFallback(CommandDimension dimension, double fallback_value,
                     double &output) const
  {
    const auto &entry = best_[static_cast<std::size_t>(dimension)];
    if (entry.has_value)
    {
      output = entry.value;
    }
    else
    {
      output = fallback_value;
    }
  }

  std::array<BestEntry, 3> best_{};
};

}  // namespace bt_executor
