/**
 * \file	main.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#include <memory>
#include <ros/ros.h>
#include "reacharm/lib/timer.h"
#include "reacharm/server/reacharm_server.h"
#include "reacharm/server/arm_provider.h"

//------------------------------------------------------------------------------
//
int main(int argc, char **argv) {
  ros::init(argc, argv, "reacharm");

  std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
  ros::Rate loop_rate(15);

  auto server = ReachArmServer();
  auto provider = ArmProvider();

  for(int i = 0; i < 360; ++i) {
    provider.SendYawAngle(static_cast<float>(i));
    provider.SendPitchAngle(static_cast<float>(i));
    MilliTimer::Sleep(10);
  }

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
