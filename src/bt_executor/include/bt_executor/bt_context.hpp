#pragma once

#include <memory>

#include "bt_executor/mission_state.hpp"
#include "bt_executor/nav_state.hpp"

namespace bt_executor
{

class DockingPhaseManager;
class HelmAdapter;
class RosIO;

struct BTContext
{
  std::shared_ptr<HelmAdapter> helm;
  std::shared_ptr<DockingPhaseManager> docking;
  std::shared_ptr<MissionState> mission;
  std::shared_ptr<NavState> nav;
  std::shared_ptr<RosIO> ros_io;
};

}  // namespace bt_executor
