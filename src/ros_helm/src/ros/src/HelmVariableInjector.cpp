#include "HelmVariableInjector.h"

#include <utility>

HelmVariableInjector::HelmVariableInjector(std::string source)
    : source_(std::move(source))
{
}

void HelmVariableInjector::queueBool(const std::string &key, bool value,
                                     double stamp)
{
  const std::string text = value ? "true" : "false";
  enqueue(HelmMsg(static_cast<char>(MsgType::Notify), key, text, stamp, source_));
}

void HelmVariableInjector::queueDouble(const std::string &key, double value,
                                       double stamp)
{
  enqueue(HelmMsg(static_cast<char>(MsgType::Notify), key, value, stamp, source_));
}

void HelmVariableInjector::queueString(const std::string &key,
                                       const std::string &value, double stamp)
{
  enqueue(HelmMsg(static_cast<char>(MsgType::Notify), key, value, stamp, source_));
}

void HelmVariableInjector::flush(HelmIvP &helm)
{
  std::list<HelmMsg> mail;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    mail.swap(pending_);
  }

  if (!mail.empty())
  {
    helm.OnNewMail(mail);
  }
}

void HelmVariableInjector::enqueue(const HelmMsg &msg)
{
  std::lock_guard<std::mutex> guard(mutex_);
  pending_.push_back(msg);
}
