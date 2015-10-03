/**
 * \file	observer_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_LIB_OBSERVER_H_
#error This file may only be included from observer.h
#endif

#include <cassert>
#include <algorithm>

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline Observer<Args_...>::Observer() noexcept : subjects_(),
                                                 subjects_mutex_() {}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline Observer<Args_...>::Observer(const Observer<Args_...> &rhs) noexcept
    : subjects_(),
      subjects_mutex_() {
  for (auto &subject : rhs.subjects_) {
    subject->Attach(*this);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline Observer<Args_...>::Observer(Subject<Args_...> &subject) noexcept
    : subjects_(),
      subjects_mutex_() {
  subject.Attach(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline Observer<Args_...>::~Observer() noexcept {
  for (const auto &subject : subjects_) {
    subject->DetachNoCallback(*this);
  }
}

//==============================================================================
// O P E R A T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Observer<Args_...>::operator=(
    const Observer<Args_...> &rhs) noexcept {
  DetachFromAllSubject();
  for (auto &subject : rhs.subjects_) {
    subject->Attach(*this);
  }
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Observer<Args_...>::DetachFromAllSubject() noexcept {
  // Do not lock the mutex here because it will be in OnSubjectDisconnected().
  for (const auto &subject : subjects_) {
    subject->Detach(*this);
  }
  subjects_.clear();
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Observer<Args_...>::OnSubjectConnected(
    Subject<Args_...> &subject) noexcept {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  auto it = std::find(subjects_.begin(), subjects_.end(), &subject);
  if (it != subjects_.end()) {
    throw std::invalid_argument("The element is already in the container.");
  } else {
    subjects_.push_back(&subject);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Observer<Args_...>::OnSubjectDisconnected(
    Subject<Args_...> &subject) {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  auto it = std::find(subjects_.begin(), subjects_.end(), &subject);
  if (it == subjects_.end()) {
    throw std::invalid_argument("The element is not in the container.");
  } else {
    subjects_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline bool Observer<Args_...>::IsAttached(
    const Subject<Args_...> &subject) const noexcept {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  return std::find(subjects_.begin(), subjects_.end(), &subject) !=
         subjects_.end();
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Observer<Args_...>::Observe(Subject<Args_...> &subject) {
  subject.Attach(*this);
}
