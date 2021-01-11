#ifndef SIMPLEBOT_DRIVER_HPP
#define SIMPLEBOT_DRIVER_HPP


#include <JetsonGPIO.h>
#include <string>
#include <cmath>
#include <boost/asio.hpp>
#include "nlohmann/json.hpp"

typedef struct {
  int pinPWM;
  int pinDirA;
  int pinDirB;
} pinInfo;

typedef struct{
  int left;
  int right;
} encoderData;

class simplebotDriver
{
public:
  simplebotDriver(pinInfo left,pinInfo right, std::string serialPort,int baudRate);
  ~simplebotDriver();
  void outputToMotor(int outputDutyLeft,int outputDutyRight);

  encoderData readEncoderFromMotor();//todo リクエスト送ったらエンコーダ更新実装（マイコン側要変更）

private:
  std::shared_ptr<GPIO::PWM> rightPWM_;
  std::shared_ptr<GPIO::PWM> leftPWM_;

  pinInfo rightPin_;
  pinInfo leftPin_;

  std::string serialPort_;
  int baudRate_;
  

  void outputToMotorDir_(int Duty,pinInfo pin);
};


#endif //SIMPLEBOT_DRIVER_HPP