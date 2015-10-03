/**
 * \file	timer.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_LIB_TIMER_H_
#define REACHARM_LIB_TIMER_H_

#include <iostream>
#include <chrono>
#include <mutex>

template <class Ut_ = std::chrono::milliseconds,
          class Tp_ = std::chrono::steady_clock>
class Timer {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Timer() noexcept;

  ~Timer() noexcept;

  //============================================================================
  // P U B L I C   S T A T I C   M E T H O D S

  /**
   * This method return the current count of the CPU.
   * The metadata can eventually be compared with another count of the CPU.
   *
   * \return the current number of count from the CPU.
   */
  static auto Now() noexcept -> int64_t;

  /**
   * Make a pause on the current calling thread.
   *
   * \param sleeping_time The time to sleep the current thread with the unit Ut_
   */
  static auto Sleep(int64_t sleeping_time) noexcept -> void;

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Start the timer
   *
   * If the timer was running, this will reset the timer before restarting it.
   */
  void Start() noexcept;

  /**
   * Pause the timer if it is not running.
   * This will throw a std::logic_error exception if the timer is not running.
   */
  void Pause();

  /**
   * Unpause a paused timer.
   * This will throw a std::logic_error exception if the timer is running.
   */
  void Unpause();

  /**
   * Reset the timer by setting both the start and the pause time to now.
   */
  void Reset() noexcept;

  /**
   * \return Either if the timer is running or being paused.
   */
  bool IsRunning() noexcept;

  /**
   * Get the difference between now and the starting time with the give unit.
   * The unit must be compatible with std::chrono units
   * -- e.g. std::chrono::seconds.
   *
   * \tparam Tp_ The unit the result will be output with.
   * \return the elapsed time from the starting point to now.
   */
  template <class Yp_ = Ut_>
  double Time() const noexcept;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * nanoseconds.
   *
   * \return The elapsed time in nanoseconds.
   */
  int64_t NanoSeconds() const noexcept;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * microseconds.
   *
   * \return The elapsed time in microseconds.
   */
  int64_t MicroSeconds() const noexcept;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * milliseconds.
   *
   * \return The elapsed time in milliseconds.
   */
  int64_t MilliSeconds() const noexcept;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * seconds.
   *
   * \return The elapsed time in seconds.
   */
  int64_t Seconds() const noexcept;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * minutes.
   *
   * \return The elapsed time in minutes.
   */
  int64_t Minutes() const noexcept;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * hours.
   *
   * \return The elapsed time in hours.
   */
  int64_t Hours() const noexcept;

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  bool is_running_ = {false};

  typename Tp_::time_point start_time_ = {};

  typename Tp_::time_point pause_time_ = {};

  mutable std::mutex member_guard_ = {};
};

using SecTimer = Timer<std::chrono::seconds, std::chrono::steady_clock>;
using MilliTimer =
    Timer<std::chrono::milliseconds, std::chrono::steady_clock>;
using MicroTimer =
    Timer<std::chrono::microseconds, std::chrono::steady_clock>;
using NanoTimer =
    Timer<std::chrono::nanoseconds, std::chrono::steady_clock>;

#include "reacharm/lib/timer_inl.h"

#endif  // REACHARM_LIB_TIMER_H_
