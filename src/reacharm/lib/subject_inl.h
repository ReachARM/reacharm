/**
 * \file	subject_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_LIB_SUBJECT_H_
#error This file may only be included from subject.h
#endif

#include <assert.h>
#include <algorithm>
#include <reacharm/lib/observer.h>

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline Subject<Args_...>::Subject() noexcept : observers_(),
                                               observers_mutex_() {}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline Subject<Args_...>::Subject(const Subject<Args_...> &rhs) noexcept
    : observers_(),
      observers_mutex_() {
  for (auto &observer : rhs.observers_) {
    observer->Observe(*this);
  }
}
//------------------------------------------------------------------------------
//
template <typename... Args_>
inline Subject<Args_...>::~Subject() noexcept {
  for (const auto &observer : observers_) {
    observer->OnSubjectDisconnected(*this);
  }
}

//==============================================================================
// O P E R A T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Subject<Args_...>::operator=(
    const Subject<Args_...> &rhs) noexcept {
  DetachAll();
  for (auto &observer : rhs.observers_) {
    Attach(*observer);
  }
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Subject<Args_...>::Attach(Observer<Args_...> &observer) {
  std::unique_lock<std::mutex> locker(observers_mutex_);

  auto it = std::find(observers_.begin(), observers_.end(), &observer);
  if (it != observers_.end()) {
    throw std::invalid_argument("The element is already in the container.");
  } else {
    observers_.push_back(&observer);
  }
  observer.OnSubjectConnected(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Subject<Args_...>::Detach(Observer<Args_...> &observer) {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  auto it = std::find(observers_.begin(), observers_.end(), &observer);
  if (it == observers_.end()) {
    throw std::invalid_argument("The element is not in the container.");
  } else {
    observers_.erase(it);
  }
  observer.OnSubjectDisconnected(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Subject<Args_...>::DetachNoCallback(Observer<Args_...> &observer) {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  auto it = std::find(observers_.begin(), observers_.end(), &observer);
  if (it == observers_.end()) {
    throw std::invalid_argument("The element is not in the container.");
  } else {
    observers_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Subject<Args_...>::DetachAll() noexcept {
  for (const auto &observer : observers_) {
    Detach(*observer);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline void Subject<Args_...>::Notify(Args_... args) noexcept {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  for (const auto &observer : observers_) {
    observer->OnSubjectNotify(*this, args...);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
inline size_t Subject<Args_...>::ObserverCount() const noexcept {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  return observers_.size();
}
