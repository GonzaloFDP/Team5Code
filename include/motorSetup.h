#include "../include/main.h"

//Ports
#define FR_MOTOR 10
#define FL_MOTOR 20
#define BR_MOTOR 1
#define BL_MOTOR 11
#define RINGTAKE 15
#define IMUNUM 2
#define CONVEYOR 14
#define FOURBAR 3
#define FORKLIFT 4

#define MR_MOTOR 12
#define ML_MOTOR 19

pros::ADIDigitalOut clampPiston ('A');

const double ticksPerDeg = 900/360;

//200*4.125*pi/60 * sqrt(2), = 60
/*const double robotSpeed = 43.196 * sqrt(2);
const double rotationSpeed = 200;*/


pros::Controller master (pros::E_CONTROLLER_MASTER);

pros::Imu inertial(IMUNUM);

//pros::Controller control (E_CONTROLLER_MASTER);
//pros::Motor Clamp(GOALCLAMP, pros::E_MOTOR_GEARSET_18);

pros::Motor FLmotor(FL_MOTOR, false);
pros::Motor FRmotor(FR_MOTOR, true);
pros::Motor BLmotor(BL_MOTOR, false);
pros::Motor BRmotor(BR_MOTOR, true);
pros::Motor MLmotor(ML_MOTOR, false);
pros::Motor MRmotor(MR_MOTOR, true);

pros::Motor Ringtake(RINGTAKE);
pros::Motor Conveyor(CONVEYOR);

pros::Motor Fourbar(FOURBAR, pros::E_MOTOR_GEARSET_36);
pros::Motor Forklift(FORKLIFT, pros::E_MOTOR_GEARSET_36, true);
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
