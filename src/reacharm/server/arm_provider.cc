/**
 * \file	arm_api.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#include "reacharm/server/arm_provider.h"
#include <ArmController/MoveBase.h>
#include <ArmController/MoveShoulder.h>
#include <ArmController/MoveElbow.h>

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ArmProvider::ArmProvider() noexcept : ndl_() {
}

//------------------------------------------------------------------------------
//
ArmProvider::~ArmProvider() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ArmProvider::SendYawAngle(float d) noexcept {
  ros::ServiceClient client = ndl_.serviceClient<ArmController::MoveBase>("move_base");

  ArmController::MoveBase base;
  base.request.angle = d;

  if(client.call(base)) {
    ROS_INFO("Sent Yaw angle to the robotic arm");
  } else {
    ROS_WARN("Cannot send angle to arm");
  }
}

//------------------------------------------------------------------------------
//
void ArmProvider::SendPitchAngle(float d) noexcept {
  ros::ServiceClient shoulder_client = ndl_.serviceClient<ArmController::MoveShoulder>("move_shoulder");
  ros::ServiceClient elbow_client = ndl_.serviceClient<ArmController::MoveElbow>("move_elbow");

  ArmController::MoveShoulder shoulder;
  shoulder.request.angle = d/2;

  ArmController::MoveElbow elbow;
  elbow.request.angle = d/2;

  shoulder_client.call(shoulder);
  elbow_client.call(elbow);
}
