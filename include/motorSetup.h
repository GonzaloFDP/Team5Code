#include "../include/main.h"


//Ports
#define FR_MOTOR 10
#define FL_MOTOR 1
#define BR_MOTOR 20
#define BL_MOTOR 11
#define RINGTAKE 15
#define GOALCLAMP 5
#define CONVEYOR 14


const double ticksPerDeg = 900/360;
//200*4.125*pi/60 * sqrt(2), = 60
/*const double robotSpeed = 43.196 * sqrt(2);
const double rotationSpeed = 200;*/


pros::Controller master (pros::E_CONTROLLER_MASTER);
 /*
ControllerButton RUp(ControllerDigital::R1);
ControllerButton RDown(ControllerDigital::R2);
ControllerButton A(ControllerDigital::A);
ControllerButton B(ControllerDigital::B);
ControllerButton X(ControllerDigital::X);
ControllerButton Y(ControllerDigital::Y);
ControllerButton left(ControllerDigital::left);
ControllerButton right(ControllerDigital::right);
ControllerButton up(ControllerDigital::up);
ControllerButton down(ControllerDigital::down);
*/

//pros::Controller control (E_CONTROLLER_MASTER);
pros::Motor Clamp(GOALCLAMP, pros::E_MOTOR_GEARSET_36);

pros::Motor FLmotor(FL_MOTOR);
pros::Motor FRmotor(FR_MOTOR, true);
pros::Motor BLmotor(BL_MOTOR);
pros::Motor BRmotor(BR_MOTOR, true);

pros::Motor Ringtake(RINGTAKE);


pros::Motor Conveyor(CONVEYOR);

/*
pros::Motor LeftIntake(LIPort, false);
pros::Motor RightIntake(RIPort, true);
pros::Motor BackRoller(LRPort, true);
pros::Motor MainRoller(RRPort, true);
pros::Imu inertial(IMUPort);

ADIEncoder yWheel('C', 'D', true);
ADIEncoder xWheel('A', 'B', false);


//Calibrate the sensors
void calibrateSensors(){
  xWheel.reset();
  yWheel.reset();
  inertial.reset();

  int timeInit = pros::millis();

  inertial.reset();
  while(inertial.is_calibrating()){
    lcd::print(1, "Calibrating");
    delay(10);
  }
  delay(2000);
  lcd::print(1, "Calibration took %f", millis() - timeInit);
}*/
