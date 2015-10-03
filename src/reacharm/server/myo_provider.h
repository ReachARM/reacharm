/**
 * \file	myo_api.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_MYO_PROVIDER_H_
#define REACHARM_MYO_PROVIDER_H_

#include <array>
#include <thread>
#include <bits/shared_ptr.h>

#include "reacharm/lib/ring_buffer.h"
#include "reacharm/lib/subject.h"

class MyoProvider: public Subject<> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MyoProvider>;

  using Buffer = ring_buffer<double>;

  enum class Movement {
    UNKNOWN = 0,
    SWAP_RIGHT,
    SWAP_LEFT,
    CLOSE_GRIP,
    TAP_FINGERS,
    OPEN_HAND
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  MyoProvider() noexcept;

  ~MyoProvider() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  double GetRollAngle() const noexcept;

  double GetPitchAngle() const noexcept;

  double GetYawAngle() const noexcept;

  bool IsSwapingRight() const noexcept;

  bool IsSwapingLeft() const noexcept;

  bool IsCloseGrip() const noexcept;

  bool IsTapingFinger() const noexcept;

  bool IsOpeningHand() const noexcept;

 private:


  double medianFilter(const Buffer& buffer) const noexcept;
  void Snapshot() noexcept;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  Buffer r_;

  Buffer p_;

  Buffer y_;

  std::thread snapshoting_thread_;

  Movement last_move_;

  static const int kBufferSize = 9;
};

#endif  // REACHARM_MYO_PROVIDER_H_
