/**
 * \file main.cpp
 */

#include <memory>
#include <ros/ros.h>

//------------------------------------------------------------------------------
//
int main(int argc, char **argv) {
  ros::init(argc, argv, "reacharm");

  std::shared_ptr<ros::NodeHandle> n =
      std::make_shared<ros::NodeHandle>();
  ros::Rate loop_rate(15);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
