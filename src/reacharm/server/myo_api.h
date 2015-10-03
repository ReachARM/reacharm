/**
 * \file	myo_api.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_MYO_API_H_
#define REACHARM_MYO_API_H_

#include <array>
#include <boost/circular_buffer.hpp>
#include "reacharm/lib/subject.h"

class MyoAPI : public Subject<> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MyoAPI>;

  using Buffer = boost::circular_buffer<double>;

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

  MyoAPI() noexcept;

  ~MyoAPI() noexcept;

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
  //==========================================================================
  // P R I V A T E   M E M B E R S

  Buffer buffer_x_;

  Buffer buffer_y_;

  Movement last_move_;

  static constexpr uint64_t kBufferSize = 10;
};

#endif  // REACHARM_MYO_API_H_
