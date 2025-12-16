#pragma once

#include <functional>
#include <list>
#include <string>
#include "HelmMsg.h"
#include "HelmUtilityFunctions.h"

namespace helm
{

using MailHandler = std::function<bool(std::list<HelmMsg> &)>;

inline MailHandler &notifyHandler()
{
  static MailHandler handler;
  return handler;
}

inline double resolveTime(double dfTime)
{
  if (dfTime < 0)
    return HelmTime();
  return dfTime;
}

inline void SetNotifyHandler(MailHandler handler)
{
  notifyHandler() = std::move(handler);
}

inline bool Notify(const std::string &sVar, const std::string &sVal,
                  const std::string &sSrcAux, double dfTime = -1)
{
  if (!notifyHandler())
    return false;

  std::list<HelmMsg> mail;
  mail.emplace_back(static_cast<char>(MsgType::Notify), sVar, sVal,
                    resolveTime(dfTime), sSrcAux);
  return notifyHandler()(mail);
}

inline bool Notify(const std::string &sVar, const std::string &sVal,
                  double dfTime = -1)
{
  return Notify(sVar, sVal, std::string(""), dfTime);
}

inline bool Notify(const std::string &sVar, double dVal,
                  const std::string &sSrcAux, double dfTime = -1)
{
  if (!notifyHandler())
    return false;

  std::list<HelmMsg> mail;
  mail.emplace_back(static_cast<char>(MsgType::Notify), sVar, dVal,
                    resolveTime(dfTime), sSrcAux);
  return notifyHandler()(mail);
}

inline bool Notify(const std::string &sVar, double dVal, double dfTime = -1)
{
  return Notify(sVar, dVal, std::string(""), dfTime);
}

inline bool Notify(const std::string &sVar, const char *sVal,
                  double dfTime = -1)
{
  return Notify(sVar, std::string(sVal), dfTime);
}

inline bool Notify(const std::string &sVar, const char *sVal,
                  const std::string &sSrcAux, double dfTime = -1)
{
  return Notify(sVar, std::string(sVal), sSrcAux, dfTime);
}

} // namespace helm

using helm::Notify;
using helm::SetNotifyHandler;

