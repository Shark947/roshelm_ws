#pragma once

#include <list>
#include <mutex>
#include <string>

#include "HelmIvP.h"
#include "HelmMsg.h"

class HelmVariableInjector
{
public:
  explicit HelmVariableInjector(std::string source = "direct_injector");

  void queueBool(const std::string &key, bool value, double stamp = -1.0);
  void queueDouble(const std::string &key, double value, double stamp = -1.0);
  void queueString(const std::string &key, const std::string &value,
                   double stamp = -1.0);

  void flush(HelmIvP &helm);

private:
  void enqueue(const HelmMsg &msg);

  std::string source_;
  std::list<HelmMsg> pending_;
  std::mutex mutex_;
};
