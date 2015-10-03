/**
 * \file	reacharm_server.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_REACHARM_SERVER_H_
#define REACHARM_REACHARM_SERVER_H_

#include <memory>
#include "reacharm/lib/observer.h"

class ReachArmServer : public Observer<> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ReachArmServer>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ReachArmServer() noexcept;

  ~ReachArmServer() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void OnSubjectNotify(Subject<> &subject) override;

};

#endif  // REACHARM_REACHARM_SERVER_H_
