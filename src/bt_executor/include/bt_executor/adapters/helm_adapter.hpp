#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include "bt_executor/mission_state.hpp"
#include "bt_executor/nav_state.hpp"

class BehaviorSet;
class HelmEngine;
class HelmReport;
class InfoBuffer;
class IvPBehavior;
class IvPDomain;
class LedgerSnap;
class VarDataPair;

namespace bt_executor
{

struct DomainEntry
{
  std::string name;
  double low{0.0};
  double high{0.0};
  unsigned int points{0};
};

struct BehaviorDefinition
{
  std::string name;
  std::string type;
  std::string update_var;
  std::vector<std::pair<std::string, std::string>> parameters;
  std::vector<std::string> extra_conditions;
  std::vector<std::string> runflags;
  std::vector<std::string> endflags;
};

struct HelmDecision
{
  double heading_deg{0.0};
  double speed{0.0};
  double depth{0.0};
  bool has_heading{false};
  bool has_speed{false};
  bool has_depth{false};
};

class HelmAdapter
{
public:
  HelmAdapter();
  ~HelmAdapter();

  bool initialize(ros::NodeHandle &private_nh, const std::string &initial_mode,
                  bool deploy_default, bool return_default);

  void syncFromState(const NavSnapshot &nav, const MissionSnapshot &mission,
                     double stamp);

  void resetActivations(double stamp);
  bool activateBehavior(const std::string &name, double stamp);
  void setUpdateVar(const std::string &var, const std::string &value,
                    double stamp);
  void setMode(const std::string &mode, double stamp);
  void setBool(const std::string &key, bool value, double stamp);
  void setString(const std::string &key, const std::string &value,
                 double stamp);

  HelmDecision solve(double stamp);

  std::vector<VarDataPair> consumeBehaviorMessages(double stamp);

  const std::unordered_map<std::string, std::string> &activationVars() const
  {
    return activation_vars_;
  }

  const std::string &domainHeadingVar() const { return heading_var_; }
  const std::string &domainSpeedVar() const { return speed_var_; }
  const std::string &domainDepthVar() const { return depth_var_; }

private:
  bool loadDomain(ros::NodeHandle &private_nh);
  bool loadInitialValues(ros::NodeHandle &private_nh);
  bool loadBehaviors(ros::NodeHandle &private_nh);
  bool buildIvPDomain();
  bool createBehaviors();

  IvPBehavior *makeBehavior(const BehaviorDefinition &definition) const;
  bool applyParameters(IvPBehavior &behavior,
                       const BehaviorDefinition &definition) const;
  void applyStandardConditions(IvPBehavior &behavior,
                               const BehaviorDefinition &definition,
                               const std::string &activation_var) const;

  void setInfoBufferDouble(const std::string &key, double value, double stamp);
  void setInfoBufferString(const std::string &key, const std::string &value,
                           double stamp);

  std::vector<DomainEntry> domain_entries_;
  std::vector<BehaviorDefinition> behavior_definitions_;
  std::map<std::string, double> initial_doubles_;

  std::unordered_map<std::string, std::string> activation_vars_;

  std::unique_ptr<IvPDomain> domain_;
  std::unique_ptr<InfoBuffer> info_buffer_;
  std::unique_ptr<LedgerSnap> ledger_snap_;
  std::unique_ptr<BehaviorSet> behavior_set_;
  std::unique_ptr<HelmEngine> helm_engine_;
  std::unique_ptr<HelmReport> last_report_;

  std::string heading_var_{"course"};
  std::string speed_var_{"speed"};
  std::string depth_var_{"depth"};
  std::string ownship_{"auh"};
};

}  // namespace bt_executor
