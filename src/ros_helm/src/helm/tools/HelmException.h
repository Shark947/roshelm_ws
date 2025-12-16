///////////////////////////////////////////////////////////////////////////
//
//   This file is part of the MOOS project
//
//   MOOS : Mission Oriented Operating Suite A suit of
//   Applications and Libraries for Mobile Robotics Research
//   Copyright (C) Paul Newman
//
//   This software was written by Paul Newman at MIT 2001-2002 and
//   the University of Oxford 2003-2013
//
//   email: pnewman@robots.ox.ac.uk.
//
//   This source code and the accompanying materials
//   are made available under the terms of the GNU Lesser Public License v2.1
//   which accompanies this distribution, and is available at
//   http://www.gnu.org/licenses/lgpl.txt
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
////////////////////////////////////////////////////////////////////////////
// HelmException.h: interface for the HelmException class.
//
//////////////////////////////////////////////////////////////////////

#ifndef HELM_EXCEPTION_H
#define HELM_EXCEPTION_H

#include <string>
#include <cstring>

/** A trivial Exception class */
class HelmException
{
public:
  HelmException();
  virtual ~HelmException();

  /** construct an exception with a string argument giving the reason
  for the exception*/
  explicit HelmException(const char *sStr);
  explicit HelmException(const std::string &s);

  char *c_str() { return m_sReason; }

  /// storage for the exception reason
  char m_sReason[100];
};

inline HelmException::HelmException() { m_sReason[0] = '\0'; }
inline HelmException::~HelmException() {}
inline HelmException::HelmException(const char *sStr)
{
  strncpy(m_sReason, sStr, sizeof(m_sReason) - 1);
  m_sReason[sizeof(m_sReason) - 1] = '\0';
}
inline HelmException::HelmException(const std::string &s)
{
  strncpy(m_sReason, s.c_str(), sizeof(m_sReason) - 1);
  m_sReason[sizeof(m_sReason) - 1] = '\0';
}

using CMOOSException = HelmException; // compatibility alias

#endif // HELM_EXCEPTION_H
