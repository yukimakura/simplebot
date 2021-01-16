#include "simplebot_hardware/simplebot_hardware.hpp"
#include "controller_manager/controller_manager.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "simplebot_hardware_node");
  ros::NodeHandle nh;

  simplebotHW robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok()){
    robot.update_joints_from_hardware(robot.getTime(), robot.getPeriod());
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write_commands_to_hardware(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}