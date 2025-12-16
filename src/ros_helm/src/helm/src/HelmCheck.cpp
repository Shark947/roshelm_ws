#include "HelmCheck.h"

#include <iostream>
#include <map>
#include "BehaviorSet.h"
#include "MBUtils.h"

HelmCheck::HelmCheck()
{
  m_host_community = "HELM_CHECK";
  m_app_name = "HelmCheck";
  m_sMOOSName = "HelmCheck";
}

bool HelmCheck::performStartupCheck(bool startup_ok, std::ostream &out)
{
  const bool info_buffer_missing = (m_info_buffer == nullptr);
  const bool domain_empty = (m_ivp_domain.size() == 0);
  const bool engine_missing = (m_hengine == nullptr);
  const bool behaviors_missing = (m_bhv_set == nullptr);
  const bool behavior_list_empty = (!m_bhv_set || m_bhv_set->size() == 0);

  out << std::boolalpha;
  out << "helm.m_info_buffer == nullptr? " << info_buffer_missing << '\n';
  out << "helm.m_ivp_domain empty? " << domain_empty << '\n';
  out << "helm.m_hengine is null? " << engine_missing << '\n';
  out << "helm.m_bhv_set is null? " << behaviors_missing << '\n';
  out << "helm.m_behaviors.size() == 0? " << behavior_list_empty << '\n';

  if (m_bhv_set)
  {
    auto warnings = m_bhv_set->getWarnings();
    for (const auto &warning : warnings)
      out << "[warning] " << warning << '\n';
  }
  out << "[check] config warning count: " << getWarningCount("config") << '\n';
  out << std::noboolalpha;

  const bool ok = startup_ok && !info_buffer_missing && !domain_empty &&
                  !engine_missing && !behavior_list_empty;

  out << "[check] startup status: " << (ok ? "true" : "false") << '\n';
  return ok;
}

void HelmCheck::printReport(std::ostream &out)
{
  m_msgs.str("");
  m_msgs.clear();
  buildReport();
  out << m_msgs.str();
}

std::vector<std::pair<std::string, double>> HelmCheck::collectDesiredDoubles() const
{
  std::map<std::string, double> desired;

  for (const auto &entry : m_outgoing_dval)
  {
    if (entry.first.rfind("DESIRED_", 0) == 0)
      desired[entry.first] = entry.second;
  }

  for (int i = 0; i < m_ivp_domain.size(); ++i)
  {
    std::string domain_var = m_ivp_domain.getVarName(i);
    std::string desired_key = "DESIRED_" + toupper(domain_var);
    if (desired_key == "DESIRED_COURSE")
      desired_key = "DESIRED_HEADING";

    if (m_helm_report.hasDecision(domain_var))
      desired[desired_key] = m_helm_report.getDecision(domain_var);
  }

  std::vector<std::pair<std::string, double>> ordered;
  ordered.insert(ordered.end(), desired.begin(), desired.end());
  return ordered;
}

bool runHelmCheck(HelmCheck &helm, bool startup_ok, std::ostream &out)
{
  return helm.performStartupCheck(startup_ok, out);
}

