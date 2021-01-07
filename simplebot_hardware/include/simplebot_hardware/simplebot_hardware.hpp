#ifndef SIMPLEBOT_HARDWARE_HPP
#define SIMPLEBOT_HARDWARE_HPP

#include "simplebot_hardware/simplebot_driver.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>

class simplebotHW : public hardware_interface::RobotHW
{
public:
    simplebotHW();
    ~simplebotHW();

    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

    void update_joints_from_hardware(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    void write_commands_to_hardware(const ros::Time& /*time*/, const ros::Duration& /*period*/);

private:
    const double PI_=3.1415926535897932;
    int oneSpinPulse_ = 500;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;

    simplebotDriver::simplebotDriver driver_;

    //以下4つはコントローラーにてupdateされると勝手に更新される
    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];
};

#endif //SIMPLEBOT_HARDWARE_HPP