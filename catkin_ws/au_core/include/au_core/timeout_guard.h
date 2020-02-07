/**
 * Timeout watchdog using a standby thread
 * Author: Vlad Didenko
 * Source:
 * https://codereview.stackexchange.com/questions/84697/timeout-watchdog-using-a-standby-thread
 */

#pragma once
#ifndef TIMEOUT_GUARD_H
#define TIMEOUT_GUARD_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace au_core {
/**
    The `clock` alias is for easy switching to `steady_clock` once Microsoft
   fixes it
*/
using clock = std::chrono::system_clock;

/**
    The `TimeoutGuard` class triggers the `alarm` callback from the
   `guard_thread` if `touch` was not called for at least the `timeout` duration.

    Because of the way the `guard_thread` sleeps, the actual detection may
   happen as late as after `timeout` + `naptime` duration. Hence it is possible
   that the alarm will not be called if the `TimeoutGuard` instance is touched
   within the 'timeout` and `timeout` + `naptime` timeframe.

    If not provided, by default the `naptime` is same as `timeout`.

    The `TimeoutGuard` is not active after construction, whicn means, that the
    `guard_thread` will block until it is activated by calling the `watch`
   method.

    The `TimeoutGuard` class is not copyable and not moveable.
*/
class TimeoutGuard {
 public:
  TimeoutGuard(clock::duration timeout, std::function<void(void)> alarm,
               clock::duration naptime);

  TimeoutGuard(clock::duration timeout, std::function<void(void)> alarm);

  ~TimeoutGuard();

  TimeoutGuard(const TimeoutGuard&) = delete;
  TimeoutGuard& operator=(const TimeoutGuard&) = delete;

  TimeoutGuard(TimeoutGuard&&) = delete;
  TimeoutGuard& operator=(TimeoutGuard&&) = delete;

  void watch();
  void touch();
  bool isIdle();

 private:
  void guard();

  clock::duration timeout;
  clock::duration naptime;
  std::function<void(void)> alarm;

  std::atomic_bool idle;
  std::atomic_bool live;

  std::atomic<clock::time_point> touched;

  std::thread guard_thread;
  std::mutex guard_mutex;
  std::condition_variable wakeup;
};
}  // namespace au_core

#endif  // TIMEOUT_GUARD_H
