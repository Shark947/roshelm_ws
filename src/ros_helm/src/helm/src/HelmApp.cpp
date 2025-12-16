#include "HelmApp.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <utility>

#include "HelmUtilityFunctions.h"
#include "MBUtils.h"
#include "Notify.h"

namespace
{
bool fileExists(const std::string &path)
{
  struct stat sb;
  return stat(path.c_str(), &sb) == 0;
}

std::string trim(const std::string &input)
{
  auto first = input.find_first_not_of(" \t");
  if (first == std::string::npos)
    return "";
  auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::string locateDefaultConfig()
{
#ifdef DEFAULT_CONFIG_PATH
  const std::string configured_path = DEFAULT_CONFIG_PATH;
  if (fileExists(configured_path))
    return configured_path;
#endif

  const std::string relative_path = "src/helm/config/startHelm.yaml";

  if (fileExists(relative_path))
    return relative_path;

  const std::string parent_path = "../" + relative_path;
  if (fileExists(parent_path))
    return parent_path;

  return relative_path;
}
}

HelmApp::HelmApp()
{
  m_iteration = 0;
  m_curr_time = 0;
  m_start_time = 0;
  m_time_warp = 1;

  m_config_path = locateDefaultConfig();

  m_last_iterate_time = 0;
  m_last_report_time = 0;
  m_last_report_time_appcast = 0;
  m_iterate_start_time = 0;
  m_term_report_interval = 0.5;

  m_term_reporting = true;
  m_new_run_warning = false;
  m_new_cfg_warning = false;

  m_app_logging = "off";
  m_deprecated_ok = false;

  m_comms_policy = "open";
  m_comms_policy_config = "open";
}

void HelmApp::Run()
{
  const double freq = (m_app_tick > 0.0) ? m_app_tick : 10.0;
  const auto interval = std::chrono::duration<double>(1.0 / freq);

  while (true)
  {
    Iterate();
    std::this_thread::sleep_for(interval);
  }
}

bool HelmApp::Iterate()
{
  m_iteration++;
  m_curr_time = HelmTime();

  // Prepare the front end of calculating the ITER_LEN
  m_iterate_start_time = m_curr_time;

  return true;
}

void HelmApp::PostReport(const std::string &directive)
{
  m_ac.setIteration(m_iteration);

  double app_freq = (m_app_tick > 0) ? m_app_tick : 0.0;
  if (app_freq > 0)
  {
    double app_gap = 1.0 / app_freq;
    double iter_len = HelmTime() - m_iterate_start_time;
    std::string app_name = MOOSToUpper((const std::string &)(m_sMOOSName));
    std::string var = app_name + "_ITER_LEN";
    Notify(var, (iter_len / app_gap));
  }

  if (m_time_warp <= 0)
    return;

  bool term_reporting = m_term_reporting;
  bool appcast_allowed = true;
  if (directive.find("noterm") != std::string::npos)
    term_reporting = false;
  else if (directive.find("doterm") != std::string::npos)
    term_reporting = true;

  if (directive.find("noappcast") != std::string::npos)
    appcast_allowed = false;

  double moos_elapsed_time_term = m_curr_time - m_last_report_time;
  double real_elapsed_time_term = moos_elapsed_time_term / m_time_warp;

  double moos_elapsed_time_appcast = m_curr_time - m_last_report_time_appcast;
  double real_elapsed_time_appcast = moos_elapsed_time_appcast / m_time_warp;

  if ((real_elapsed_time_term < m_term_report_interval) &&
      (real_elapsed_time_appcast < m_term_report_interval))
    return;

  bool appcast_pending = false;
  if (appcast_allowed)
    appcast_pending = appcastRequested() || (m_iteration < 2);

  if (!term_reporting && !appcast_pending)
    return;
  if (!appcast_pending && (real_elapsed_time_term < m_term_report_interval))
    return;
  if (!term_reporting && (real_elapsed_time_appcast < m_term_report_interval))
    return;

  m_msgs.clear();
  m_msgs.str("");

  bool report_built = buildReport();
  if (!report_built)
    return;

  m_ac.msg(m_msgs.str());

  if (term_reporting)
  {
    m_last_report_time = m_curr_time;
    std::cout << "\n\n\n\n\n\n\n\n\n";
    std::cout << m_ac.getFormattedString();
  }

  if (appcast_pending)
  {
    m_new_run_warning = false;
    m_new_cfg_warning = false;
    m_last_report_time_appcast = m_curr_time;
  }
}

bool HelmApp::OnStartUp()
{
  m_start_time = HelmTime();
  m_time_warp = GetHelmTimeWarp();
  return true;
}

bool HelmApp::OnNewMail(std::list<CMOOSMsg> &NewMail)
{
  m_curr_time = HelmTime();

  for (auto p = NewMail.begin(); p != NewMail.end();)
  {
    CMOOSMsg &msg = *p;
    if (msg.GetKey() == "APPCAST_REQ")
    {
      handleMailAppCastRequest(msg.GetString());
      p = NewMail.erase(p);
    }
    else if (msg.GetKey() == "COMMS_POLICY")
    {
      std::string sval = msg.GetString();
      MOOSToLower(sval);
      bool ok = handleMailCommsPolicy(sval);
      if (!ok)
        reportRunWarning("Unhandled COMMS_POLICY:" + sval);
      p = NewMail.erase(p);
    }
    else if (msg.GetKey() == "TERM_REPORT_INTERVAL")
    {
      double dval = msg.GetDouble();
      m_term_report_interval = dval;
      if (m_term_report_interval < 0.4)
        m_term_report_interval = 0.4;
      p = NewMail.erase(p);
    }
    else if (msg.GetKey() == "_async_timing")
      p = NewMail.erase(p);
    else
      ++p;
  }
  return true;
}

void HelmApp::reportEvent(const std::string &str)
{
  double timestamp = m_curr_time - m_start_time;
  m_ac.event(str, timestamp);
}

void HelmApp::reportConfigWarning(const std::string &str)
{
  if (str.empty())
    return;

  m_new_cfg_warning = true;
  m_ac.cfgWarning(str);
  std::cerr << "[config warning] " << str << std::endl;
}

void HelmApp::reportUnhandledConfigWarning(const std::string &orig)
{
  if (orig.empty())
    return;

  std::string orig_copy = orig;
  std::string param = MOOSToUpper(MOOSChomp(orig_copy, "="));
  MOOSTrimWhiteSpace(param);
  if ((param == "APPTICK") || (param == "APP_LOGGING") ||
      (param == "MAXAPPTICK") || (param == "TERM_REPORT_INTERVAL") ||
      (param == "COMMSTICK") || (param == "MAX_APPCAST_EVENTS") ||
      (param == "DEPRECATED_OK"))
    return;

  reportConfigWarning("Unhandled config line: " + orig);
}

bool HelmApp::reportRunWarning(const std::string &str)
{
  if (str.empty())
    return false;
  m_new_run_warning = true;
  m_ac.runWarning(str);
  return false;
}

void HelmApp::retractRunWarning(const std::string &str)
{
  if (m_ac.getRunWarningCount() == 0)
    return;

  bool successfully_retracted = m_ac.retractRunWarning(str);

  if (successfully_retracted)
    m_new_run_warning = true;
}

unsigned int HelmApp::getWarningCount(const std::string &filter) const
{
  unsigned int total = 0;
  if ((filter == "all") || (filter == "config"))
    total += m_ac.getCfgWarningCount();

  if ((filter == "all") || (filter == "run"))
    total += m_ac.getRunWarningCount();

  return total;
}

void HelmApp::registerSingleVariable(const std::string &var, double freq)
{
  (void)freq;
  if (!var.empty())
    m_registered_vars.insert(var);
}

std::vector<std::string> HelmApp::collectAdditionalRegisterVariables() const
{
  return {};
}

void HelmApp::RegisterVariables()
{
  for (const auto &var : loadRegisterVariables())
    registerSingleVariable(var);

  for (const auto &extra : collectAdditionalRegisterVariables())
    registerSingleVariable(extra);
}

void HelmApp::handleMailAppCastRequest(const std::string &str)
{
  std::string s_key;
  std::string s_duration;
  std::string s_thresh = "any";

  std::string request = str;
  while (!request.empty())
  {
    std::string pair = MOOSChomp(request, ",");
    std::string param = MOOSToUpper(MOOSChomp(pair, "="));
    std::string value = pair;
    MOOSTrimWhiteSpace(param);
    MOOSTrimWhiteSpace(value);

    if (param == "NODE")
    {
      if ((value != m_host_community) && (value != "all"))
        return;
    }
    else if (param == "APP")
    {
      if ((value != m_app_name) && (value != "all"))
        return;
    }
    else if (param == "DURATION")
      s_duration = value;
    else if (param == "THRESH")
      s_thresh = value;
    else if (param == "KEY")
    {
      s_key = value;
    }
  }

  if (s_key.empty())
    return;

  double d_duration = atof(s_duration.c_str());
  d_duration = (d_duration < 0) ? 0 : d_duration;
  d_duration = (d_duration > 30) ? 30 : d_duration;

  m_map_bcast_duration[s_key] = d_duration;
  m_map_bcast_tstart[s_key] = m_curr_time;
  m_map_bcast_thresh[s_key] = s_thresh;
}

bool HelmApp::appcastRequested()
{
  bool requested = false;

  for (auto &entry : m_map_bcast_duration)
  {
    const std::string &key = entry.first;
    double duration = entry.second;
    double elapsed = m_curr_time - m_map_bcast_tstart[key];

    if (elapsed < duration)
    {
      if (m_map_bcast_thresh[key] == "any")
        requested = true;
      else if ((m_map_bcast_thresh[key] == "run_warning") && m_new_run_warning)
        requested = true;
    }
  }
  if (m_new_cfg_warning)
    requested = true;

  return requested;
}

bool HelmApp::handleMailCommsPolicy(const std::string &str)
{
  if ((str != "open") && (str != "lean") && (str != "dire"))
    return false;

  if (str == "open")
  {
    if ((m_comms_policy_config == "lean") || (m_comms_policy_config == "dire"))
      return true;
  }

  if (str == "lean")
  {
    if (m_comms_policy_config == "dire")
      return false;
  }

  m_comms_policy = str;

  return true;
}

void HelmApp::AppCastState::event(const std::string &str, double timestamp)
{
  std::string formatted = str;
  if (timestamp >= 0)
  {
    std::stringstream ss;
    ss << "[" << std::fixed << std::setprecision(2) << timestamp << "]: " << str;
    formatted = ss.str();
  }
  m_events.push_back(formatted);
  if (m_events.size() > m_max_events)
    m_events.pop_front();
}

void HelmApp::AppCastState::cfgWarning(const std::string &str)
{
  if (m_config_warnings.size() > m_max_config_warnings)
    return;

  m_config_warnings.push_back(str);
}

void HelmApp::AppCastState::runWarning(const std::string &str)
{
  if (m_map_run_warnings.size() < m_max_run_warnings)
    m_map_run_warnings[str]++;
  else
    m_map_run_warnings["Other Run Warnings"]++;

  ++m_cnt_run_warnings;
}

bool HelmApp::AppCastState::retractRunWarning(const std::string &str)
{
  if (m_map_run_warnings.count(str) == 0)
    return false;

  unsigned int count = m_map_run_warnings[str];
  m_map_run_warnings.erase(str);

  if (m_cnt_run_warnings >= count)
    m_cnt_run_warnings -= count;
  else
    m_cnt_run_warnings = 0;

  return true;
}

std::string HelmApp::AppCastState::getFormattedString(bool with_header) const
{
  std::ostringstream os;
  if (with_header)
  {
    os << "proc=" << m_proc_name << "  iter=" << m_iteration;
    if (!m_node_name.empty())
      os << "  node=" << m_node_name;
    os << "\n";
  }

  if (!m_messages.empty())
    os << m_messages << "\n";

  if (!m_config_warnings.empty())
  {
    os << "Configuration Warnings (" << m_config_warnings.size() << "):\n";
    for (const auto &warning : m_config_warnings)
      os << "  - " << warning << "\n";
  }

  if (!m_events.empty())
  {
    os << "Events (" << m_events.size() << "):\n";
    for (const auto &event : m_events)
      os << "  - " << event << "\n";
  }

  if (!m_map_run_warnings.empty())
  {
    os << "Run Warnings (" << m_cnt_run_warnings << "):\n";
    for (const auto &entry : m_map_run_warnings)
      os << "  - " << entry.second << ":" << entry.first << "\n";
  }

  return os.str();
}

std::vector<std::string> HelmApp::loadRegisterVariables() const
{
  std::vector<std::string> variables;
  std::string path = m_register_config_path;
  if (path.empty())
  {
    if (!m_config_dir.empty())
      path = m_config_dir + "/registerVariables.yaml";
    else
      path = "registerVariables.yaml";
  }

  std::ifstream infile(path);
  if (!infile.is_open())
    return variables;

  bool in_variables = false;
  std::string line;
  while (std::getline(infile, line))
  {
    std::string cleaned = trim(line);
    if (cleaned.empty() || cleaned[0] == '#')
      continue;

    if (cleaned == "variables:" || cleaned == "variables:|")
    {
      in_variables = true;
      continue;
    }

    if (!in_variables)
      continue;

    if (cleaned.rfind("-", 0) == 0)
    {
      std::string var = trim(cleaned.substr(1));
      if (!var.empty())
        variables.push_back(var);
    }
  }

  return variables;
}

