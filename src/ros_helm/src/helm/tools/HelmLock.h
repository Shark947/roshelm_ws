#pragma once

#include <mutex>

// Simple mutex wrapper that replaces the original MOOS lock utilities
// with standard C++ primitives.
class HelmLock
{
public:
  HelmLock() = default;
  HelmLock(const HelmLock &) = delete;
  HelmLock &operator=(const HelmLock &) = delete;

  void Lock() { m_mutex.lock(); }
  void UnLock() { m_mutex.unlock(); }

private:
  std::mutex m_mutex;
};

class HelmScopedLock
{
public:
  explicit HelmScopedLock(HelmLock &lock) : m_lock(lock) { m_lock.Lock(); }
  ~HelmScopedLock() { m_lock.UnLock(); }

  HelmScopedLock(const HelmScopedLock &) = delete;
  HelmScopedLock &operator=(const HelmScopedLock &) = delete;

private:
  HelmLock &m_lock;
};

using CMOOSLock = HelmLock;
namespace Helm
{
using ScopedLock = HelmScopedLock;
}

