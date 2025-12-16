#pragma once

#include <iosfwd>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "HelmIvP.h"

class HelmCheck : public HelmIvP
{
public:
  HelmCheck();

  bool performStartupCheck(bool startup_ok, std::ostream &out);
  void printReport(std::ostream &out);

  std::map<std::string, double> getOutgoingDoubles() const { return m_outgoing_dval; }
  std::map<std::string, std::string> getOutgoingStrings() const { return m_outgoing_sval; }

  std::string getStatus() const { return helmStatus(); }
  bool hasControl() const { return m_has_control; }
  const STRING_LIST &getStartupParams() const { return m_startup_params; }

  std::vector<std::pair<std::string, double>> collectDesiredDoubles() const;
};

bool runHelmCheck(HelmCheck &helm, bool startup_ok, std::ostream &out);

