/**
 * \file	observer.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_LIB_OBSERVER_H_
#define REACHARM_LIB_OBSERVER_H_

#include <type_traits>
#include <vector>
#include <functional>
#include <mutex>

#include "reacharm/lib/subject.h"

template <typename... Args_>
class Observer {
  // The callback on the Observer class are private by default.
  // We don't want other class than the Subject to be able to call the
  // OnSubjectXXX() method.
  friend class Subject<Args_...>;

 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Observer() noexcept;

  explicit Observer(Subject<Args_...> &subject) noexcept;

  /**
   * Copy ctor of an observer. This will attach this instance of an Observer
   * to all listened subject of the passed observer.
   *
   * \param rhs The Observer base used to create this instance.
   */
  explicit Observer(const Observer<Args_...> &rhs) noexcept;

  virtual ~Observer() noexcept;

  //============================================================================
  // P U B L I C   O P E R A T O R S

  /**
   * Copy ctor of an observer. This will attach this instance of an Observer
   * to all listened subject of the passed observer.
   *
   * \param rhs The Observer base used to create this instance.
   */
  void operator=(const Observer<Args_...> &rhs) noexcept;

  //============================================================================
  // P U B L I C  M E T H O D S

  void Observe(Subject<Args_...> &subject);

  bool IsAttached(const Subject<Args_...> &subject) const noexcept;

  void DetachFromAllSubject() noexcept;

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * This method is used if you want to use the observer without delegates.
   * If so, then the method will be called instead of the delegate.
   * If not, then simply override this and do nothing.
   */
  virtual void OnSubjectNotify(Subject<Args_...> &subject,
                               Args_... args) = 0;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  virtual void OnSubjectConnected(Subject<Args_...> &subject) noexcept;

  virtual void OnSubjectDisconnected(Subject<Args_...> &subject);

  //============================================================================
  // P R I V A T E   M E M B E R S

  std::vector<Subject<Args_...> *> subjects_;

  mutable std::mutex subjects_mutex_;
};

#include <reacharm/lib/observer_inl.h>

#endif  // REACHARM_LIB_OBSERVER_H_
