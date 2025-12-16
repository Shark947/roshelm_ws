#pragma once

#include <list>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "HelmMsg.h"
#include "HelmUtilityFunctions.h"

class HelmApp
{
public:
  HelmApp();
  virtual ~HelmApp() = default;

  void Run();

  virtual bool Iterate();
  virtual bool OnNewMail(std::list<HelmMsg> &NewMail);
  virtual bool OnStartUp();
  virtual bool buildReport() { return false; }
  virtual bool deprecated() { return false; }

  void setHostCommunity(const std::string &community) { m_host_community = community; }
  void setAppName(const std::string &name)
  {
    m_app_name = name;
    m_sMOOSName = name;
  }
  void setConfigPath(const std::string &path) { m_config_path = path; }
  void setConfigDirectory(const std::string &path) { m_config_dir = path; }
  void setRegisterVariablesPath(const std::string &path)
  {
    m_register_config_path = path;
  }

  void setStartupParameters(const STRING_LIST &params) { m_startup_params = params; }
  void setAppTick(double app_tick) { m_app_tick = app_tick; }
  void setCommsTick(double comms_tick) { m_comms_tick = comms_tick; }

protected:
  void RegisterVariables();
  void PostReport(const std::string &directive = "");
  void reportEvent(const std::string &);
  void reportConfigWarning(const std::string &);
  void reportUnhandledConfigWarning(const std::string &);
  bool reportRunWarning(const std::string &);
  void retractRunWarning(const std::string &);
  unsigned int getWarningCount(const std::string &) const;

  std::string commsPolicy() const { return m_comms_policy; }

  virtual void registerSingleVariable(const std::string &var, double freq = 0.0);
  virtual std::vector<std::string> collectAdditionalRegisterVariables() const;

protected:
  unsigned int m_iteration;
  double m_curr_time;
  double m_start_time;
  double m_time_warp;
  double m_last_iterate_time;
  double m_iterate_start_time;
  double m_last_report_time;
  double m_term_report_interval;
  bool m_term_reporting;
  bool m_deprecated_ok;
  std::string m_deprecated_alt;

  std::stringstream m_msgs;
  std::stringstream m_cout;

  std::string m_host_community;
  std::string m_app_name{"HELM_APP"};
  std::string m_sMOOSName{"HELM_APP"};

  double m_app_tick{0.0};
  double m_comms_tick{0.0};
  std::string m_config_path;

  STRING_LIST m_startup_params;
  std::set<std::string> m_registered_vars;
  std::string m_config_dir;
  std::string m_register_config_path;

private:
  class AppCastState
  {
  public:
    void msg(const std::string &str) { m_messages = str; }
    void event(const std::string &str, double timestamp = -1.0);
    void runWarning(const std::string &);
    bool retractRunWarning(const std::string &);
    void cfgWarning(const std::string &);
    void setProcName(const std::string &name) { m_proc_name = name; }
    void setNodeName(const std::string &name) { m_node_name = name; }
    void setIteration(unsigned int value) { m_iteration = value; }
    void setMaxEvents(unsigned int value) { m_max_events = value; }
    void setMaxRunWarnings(unsigned int value) { m_max_run_warnings = value; }
    std::string getFormattedString(bool with_header = true) const;
    unsigned int getIteration() const { return m_iteration; }
    unsigned int getRunWarningCount() const { return m_cnt_run_warnings; }
    std::string::size_type getCfgWarningCount() const
    {
      return m_config_warnings.size();
    }

  private:
    std::string m_proc_name;
    std::string m_node_name;
    unsigned int m_max_events{8};
    unsigned int m_max_run_warnings{10};
    unsigned int m_max_config_warnings{100};
    unsigned int m_iteration{0};
    std::string m_messages;
    std::vector<std::string> m_config_warnings;
    std::map<std::string, unsigned int> m_map_run_warnings;
    unsigned int m_cnt_run_warnings{0};
    std::list<std::string> m_events;
  };

  void handleMailAppCastRequest(const std::string &str);
  bool handleMailCommsPolicy(const std::string &str);
  bool appcastRequested();

  std::vector<std::string> loadRegisterVariables() const;

private:
  AppCastState m_ac;
  bool m_new_run_warning;
  bool m_new_cfg_warning;

  double m_last_report_time_appcast;
  std::string m_app_logging;
  std::string m_app_logging_info;
  std::string m_comms_policy;
  std::string m_comms_policy_config;

  std::map<std::string, double> m_map_bcast_duration;
  std::map<std::string, double> m_map_bcast_tstart;
  std::map<std::string, std::string> m_map_bcast_thresh;
};

