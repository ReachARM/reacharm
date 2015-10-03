/**
 * \file	timer_inl.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_LIB_TIMER_H_
#error This file may only be included from timer.h
#endif

#include <math.h>
#include <thread>

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline Timer<Up_, Tp_>::Timer() noexcept {}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline Timer<Up_, Tp_>::~Timer() noexcept {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline void Timer<Up_, Tp_>::Start() noexcept {
  Reset();
  std::lock_guard<std::mutex> guard(member_guard_);
  is_running_ = true;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline void Timer<Up_, Tp_>::Reset() noexcept {
  std::lock_guard<std::mutex> guard(member_guard_);
  start_time_ = Tp_::now();
  pause_time_ = start_time_;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline void Timer<Up_, Tp_>::Pause() {
  std::lock_guard<std::mutex> guard(member_guard_);
  if (!is_running_) {
    guard.~lock_guard();
    throw std::logic_error("The timer is not running");
  }
  pause_time_ = Tp_::now();
  is_running_ = false;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline void Timer<Up_, Tp_>::Unpause() {
  std::lock_guard<std::mutex> guard(member_guard_);
  if (is_running_) {
    guard.~lock_guard();
    throw std::logic_error("The timer is running");
  }
  start_time_ += Tp_::now() - pause_time_;
  is_running_ = true;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline bool Timer<Up_, Tp_>::IsRunning() noexcept {
  std::lock_guard<std::mutex> guard(member_guard_);
  return is_running_;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
template <typename Yp_>
inline double Timer<Up_, Tp_>::Time() const noexcept {
  std::lock_guard<std::mutex> guard(member_guard_);
  auto time = Tp_::now() - start_time_;
  if (!is_running_) {
    time = pause_time_ - start_time_;
  }
  auto period = static_cast<double>(Yp_::period::num) /
                static_cast<double>(Yp_::period::den);
  return static_cast<double>(std::chrono::duration_cast<Yp_>(time).count() *
                             period);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline int64_t Timer<Up_, Tp_>::Now() noexcept {
  return std::chrono::duration_cast<Up_>(Tp_::now().time_since_epoch()).count();
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline void Timer<Up_, Tp_>::Sleep(int64_t sleeping_time)
    noexcept {
  std::this_thread::sleep_for(Up_(sleeping_time));
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline int64_t
Timer<Up_, Tp_>::NanoSeconds() const noexcept {
  return static_cast<int64_t>(Time<std::chrono::nanoseconds>() * 1000000000);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline int64_t
Timer<Up_, Tp_>::MicroSeconds() const noexcept {
  return static_cast<int64_t>(Time<std::chrono::microseconds>() * 1000000);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline int64_t
Timer<Up_, Tp_>::MilliSeconds() const noexcept {
  return static_cast<int64_t>(Time<std::chrono::milliseconds>() * 1000);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline int64_t Timer<Up_, Tp_>::Seconds() const noexcept {
  return static_cast<int64_t>(Time<std::chrono::seconds>());
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline int64_t Timer<Up_, Tp_>::Minutes() const noexcept {
  return static_cast<int64_t>(Time<std::chrono::minutes>() / 60);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
inline int64_t Timer<Up_, Tp_>::Hours() const noexcept {
  return static_cast<int64_t>(Time<std::chrono::hours>() / 3600);
}
