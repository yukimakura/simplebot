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

    pinInfo rightPin = {32,21,22};
    pinInfo leftPin = {33,23,24};
    driver_ = new simplebotDriver::simplebotDriver(leftPin,leftPin);
}

simplebotHW::~simplebotHW(){
    delete driver_;
}



void simplebotHW::update_joints_from_hardware(const ros::Time& time, const ros::Duration& period) {
    simplebotDriver::encoderData encData = driver_.readEncoderFromMotor();
    vel_[0] = ((double)encData.left / (double)oneSpinPulse_) * 2.0 * PI_;
    vel_[1] = ((double)encData.right / (double)oneSpinPulse_) * 2.0 * PI_;

    pos_[0] += vel_[0];
    pos_[1] += vel_[1];
    
    ROS_INFO_STREAM("feedback from joints: " << vel_[0] << ", " << vel_[1]);
    
}

void simplebotHW::write_commands_to_hardware(const ros::Time& time, const ros::Duration& period) {
    // RobotHWはrad/sで送ってくる！
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
    //todo ハードウェアへの出力を実装
}
