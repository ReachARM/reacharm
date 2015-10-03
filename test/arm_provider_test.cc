/**
 * \file	arm_provider_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#include <gtest/gtest.h>
#include "reacharm/lib/timer.h"

auto provider = ArmProvider();

TEST(AngleMoves, AllTest) {
  for(int i = 0; i < 360; ++i) {
    provider.SendYawAngle(static_cast<float>(i));
    provider.SendPitchAngle(static_cast<float>(i));
    MilliTimer::Sleep(10);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
