/**
 * \file	myo_api.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#include "reacharm/server/myo_api.h"

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MyoAPI::MyoAPI() noexcept : buffer_x_(kBufferSize),
                            buffer_y_(kBufferSize),
                            last_move_(Movement::UNKNOWN) {}

//------------------------------------------------------------------------------
//
MyoAPI::~MyoAPI() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
double MyoAPI::GetRollAngle() const noexcept {}

//------------------------------------------------------------------------------
//
double MyoAPI::GetPitchAngle() const noexcept {}

//------------------------------------------------------------------------------
//
double MyoAPI::GetYawAngle() const noexcept {}

//------------------------------------------------------------------------------
//
bool MyoAPI::IsSwapingRight() const noexcept {
  return last_move_ == Movement::SWAP_RIGHT;
}

//------------------------------------------------------------------------------
//
bool MyoAPI::IsSwapingLeft() const noexcept {
  return last_move_ == Movement::SWAP_LEFT;
}

//------------------------------------------------------------------------------
//
bool MyoAPI::IsCloseGrip() const noexcept {
  return last_move_ == Movement::CLOSE_GRIP;
}

//------------------------------------------------------------------------------
//
bool MyoAPI::IsTapingFinger() const noexcept {
  return last_move_ == Movement::TAP_FINGERS;
}

//------------------------------------------------------------------------------
//
bool MyoAPI::IsOpeningHand() const noexcept {
  return last_move_ == Movement::OPEN_HAND;
}
