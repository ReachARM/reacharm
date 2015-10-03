/**
 * \file	myo_api.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#include "myo_provider.h"

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MyoProvider::MyoProvider() noexcept : r_(Buffer(kBufferSize)),
                            p_(Buffer(kBufferSize)),
                            y_(Buffer(kBufferSize)),
                            snapshoting_thread_(std::thread()),
                            last_move_(Movement::UNKNOWN)   {}

//------------------------------------------------------------------------------
//
MyoProvider::~MyoProvider() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
double MyoProvider::GetRollAngle() const noexcept {
  return medianFilter(r_);
}

//------------------------------------------------------------------------------
//
double MyoProvider::GetPitchAngle() const noexcept {
  return medianFilter(p_);
}

//------------------------------------------------------------------------------
//
double MyoProvider::GetYawAngle() const noexcept {
  return medianFilter(y_);
}

//------------------------------------------------------------------------------
//
bool MyoProvider::IsSwapingRight() const noexcept {
  return last_move_ == Movement::SWAP_RIGHT;
}

//------------------------------------------------------------------------------
//
bool MyoProvider::IsSwapingLeft() const noexcept {
  return last_move_ == Movement::SWAP_LEFT;
}

//------------------------------------------------------------------------------
//
bool MyoProvider::IsCloseGrip() const noexcept {
  return last_move_ == Movement::CLOSE_GRIP;
}

//------------------------------------------------------------------------------
//
bool MyoProvider::IsTapingFinger() const noexcept {
  return last_move_ == Movement::TAP_FINGERS;
}

//------------------------------------------------------------------------------
//
bool MyoProvider::IsOpeningHand() const noexcept {
  return last_move_ == Movement::OPEN_HAND;
}

//------------------------------------------------------------------------------
//
void MyoProvider::Snapshot() noexcept {
  
}

//------------------------------------------------------------------------------
//
double MyoProvider::medianFilter(const Buffer &buffer) const noexcept {
  return buffer.sort()[buffer.size()/2];
}