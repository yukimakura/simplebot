#include "simplebot_hardware/simplebot_driver.hpp"
using namespace simplebotDriver;


simplebotDriver::simplebotDriver(pinInfo right,pinInfo left,std::string serialPort,int baudRate)
    :rightPin_(right),leftPin_(left),serialPort_(serialPort),baudRate_(baudRate)
{
    // Pin Setup. 
	// Board pin-numbering scheme
	GPIO::setmode(GPIO::BOARD);

	// set pin as an output pin with optional initial state of HIGH
	GPIO::setup(right.pinPWM, GPIO::OUT, GPIO::LOW);
    GPIO::setup(right.pinDirA, GPIO::OUT, GPIO::LOW);
    GPIO::setup(right.pinDirB, GPIO::OUT, GPIO::LOW);

    GPIO::setup(left.pinPWM, GPIO::OUT, GPIO::LOW);
    GPIO::setup(left.pinDirA, GPIO::OUT, GPIO::LOW);
    GPIO::setup(left.pinDirB, GPIO::OUT, GPIO::LOW);

    
    rightPWM_ = new GPIO::PWM(right.pinPWM, 50);
    leftPWM_ = new GPIO::PWM(left.pinPWM, 50);
    rightPWM_.start(0);
    leftPWM_.start(0);

    
}

simplebotDriver::~simplebotDriver(){
    
    rightPWM_.stop();
    leftPWM_.stop();
    GPIO::cleanup();

    delete rightPWM_;
    delete leftPWM_;
}
void simplebotDriver::outputToMotor(int outputDutyLeft,int outputDutyRight){
    outputToMotorDir_(outputDutyLeft,leftPin_);
    outputToMotorDir_(outputDutyRight,rightPin_);

    leftPWM_.ChangeDutyCycle(std::abs(outputDutyLeft));
    rightPWM_.ChangeDutyCycle(std::abs(outputDutyRight));

}

encoderData simplebotDriver::readEncoderFromMotor(){
//todo リクエスト送ったらエンコーダ更新実装（マイコン側要変更）
    boost::asio::io_service io;

    boost::asio::serial_port serial(io, serialPort_);
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudRate_));

    boost::asio::write(serial, boost::asio::buffer("request\n"));

    // serial から response_buf に '\n' まで読み込む
    boost::asio::streambuf response_buf;
    boost::asio::read_until(serial, response_buf, '\n');
    auto json = nlohmann::json::parse(boost::asio::buffer_cast<const char*>(response_buf.data()));
    
    encoderData retEncData = {json["left"],json["right"]};
    return retEncData;
}

void outputToMotorDir_(int Duty,pinInfo pin){
    if(Duty < 0){
        GPIO::output(pin.pinDirA, GPIO::HIGH);
        GPIO::output(pin.pinDirB, GPIO::LOW);
    }else{
        GPIO::output(pin.pinDirA, GPIO::LOW);
        GPIO::output(pin.pinDirB, GPIO::HIGH);
    }
}
