/**
 * \file	arm_api.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_ARM_API_H_
#define REACHARM_ARM_API_H_

#include <memory>
#include "reacharm/lib/service_client_manager.h"

class ArmProvider {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ArmProvider>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ArmProvider() noexcept;

  ~ArmProvider() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void SendYawAngle(float d) noexcept;

  void SendPitchAngle(float d) noexcept;

  ros::NodeHandle ndl_;
};

#endif  // REACHARM_ARM_API_H_
