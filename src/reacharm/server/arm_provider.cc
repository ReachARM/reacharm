/**
 * \file	arm_api.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#include "arm_provider.h"

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ArmProvider::ArmProvider() noexcept : ServiceClientManager() {}

//------------------------------------------------------------------------------
//
ArmProvider::~ArmProvider() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ArmProvider::SendYawAngle(double d, const MotorID &id) const noexcept {}

//------------------------------------------------------------------------------
//
void ArmProvider::SendRollAngle(double d, const MotorID &id) const noexcept {}

//------------------------------------------------------------------------------
//
void ArmProvider::SendPitchAngle(double d, const MotorID &id) const noexcept {}
