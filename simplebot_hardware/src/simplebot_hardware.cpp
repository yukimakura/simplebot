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
    driver_ = std::make_shared<simplebotDriver>(leftPin,rightPin,"/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066DFF495649657867072447-if02",38400);

}

simplebotHW::~simplebotHW(){
    driver_->outputToMotor(0,0);
}
void simplebotHW::update_joints_from_hardware(const ros::Time& time, const ros::Duration& period) {
    encoderData encData = driver_->readEncoderFromMotor();
    //ROS_INFO("leftenc:%d,rightenc:%d",encData.left,encData.right);
    //ROS_INFO("onespinpulse:%d,left / onespin : %f",oneSpinPulse_,((double)(encData.left) / (double)(oneSpinPulse_)));
    vel_[0] = ((double)(encData.left) / (double)(oneSpinPulse_)) * 2.0 * PI_;
    vel_[1] = ((double)(encData.right) / (double)(oneSpinPulse_)) * 2.0 * PI_;
    //ROS_INFO("leftvel:%f,rightvel:%f",vel_[0],vel_[1]);
    pos_[0] += vel_[0];
    pos_[1] += vel_[1];
    
}

void simplebotHW::write_commands_to_hardware(const ros::Time& time, const ros::Duration& period) {
    // RobotHWはrad/sで送ってくる！
    //ROS_INFO("leftpwm:%d,rightpwm:%d",rad2pwm_(cmd_[0]),rad2pwm_(cmd_[1]));
    driver_->outputToMotor(rad2pwm_(cmd_[0]),rad2pwm_(cmd_[1]));
}

int simplebotHW::rad2pwm_(double cmd){
    int ret;
    ret = ((double)oneSpinPulse_/((double)maxSpeedPulse_* 2.0 * PI_)) * 100.0 * cmd ;
    if((((double)maxSpeedPulse_/(double)oneSpinPulse_) * 2.0 * PI_) < cmd){
        ret = 100;
    }
    return ret;
}
