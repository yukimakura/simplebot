#include "simplebot_hardware/simplebot_hardware.hpp"

simplebotHW::simplebotHW(){
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    hardware_interface::JointStateHandle state_handle_1("wheel_left_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("wheel_right_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("wheel_left_joint"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("wheel_right_joint"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
}


void simplebotHW::read(const ros::Time& time, const ros::Duration& period) {
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
}

void simplebotHW::write(const ros::Time& time, const ros::Duration& period) {
    //Update pos_ and vel_
}
