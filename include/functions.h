#include "main.h"
#include "motorSetup.h"

//okapi::Controller master;
//Useful Constants
const double wheelCircumfrence = 3.25 * M_PI;

int countr = 0;
std::string autons[4] = {"Red1", "Red2", "Blue1", "Blue2"};
int size = 4;//*(&autons + 1) - autons;

void screenPrintString(int e, int o, std::string i){
  master.print(e,o,i.c_str());
}

void screenPrintInt(int e, int o, double i){
  master.print(e, o, "%i", int(i));
}

double distanceToTicks(double distance){
  double ticks = distance;
  ticks /= (wheelCircumfrence*3);
  ticks *= 900;
  return ticks;
}

void autonSelector(){
  master.clear();
  pros::delay(200);
  while(true){
    master.clear();
    pros::delay(100);
    screenPrintString(2, 1, autons[countr].c_str());
    pros::delay(100);
     if(master.get_digital(DIGITAL_RIGHT)){

       countr = (countr + 1 + size) % size;

     } else if(master.get_digital(DIGITAL_LEFT)){

       countr = (countr - 1 + size) % size;

     } else if(master.get_digital(DIGITAL_A)){

       pros::delay(200);

       if(master.get_digital(DIGITAL_A)){

         break;

       }
     }
   }
   master.rumble("..");
}

void opDriver(double left, double right){
  //Calculates speed of wheels for driver control
	FLmotor.move_velocity(left);
	FRmotor.move_velocity(right);
	BLmotor.move_velocity(left);
	BRmotor.move_velocity(right);
}

void goalClampMovement(bool upOrDown){
  if (upOrDown){
    Clamp.move_velocity(100);
  } else if (upOrDown == false){
    Clamp.move_velocity(-100);
  } else {
    Clamp.move_velocity(0);
  }
}

void ringtakeMovement(bool upOrDown){
  if (upOrDown){
    Ringtake.move_velocity(100);
  } else if (upOrDown == false){
    Ringtake.move_velocity(-100);
  } else {
    Ringtake.move_velocity(0);
  }
}

void conveyorMovement(bool upOrDown){
  if (upOrDown){
    Conveyor.move_velocity(100);
  } else if (upOrDown == false){
    Conveyor.move_velocity(-100);
  } else {
    Conveyor.move_velocity(0);
  }
}

double platformMode(double currentMode){
  if(currentMode == 1){
    return 0.6;
  } else if (currentMode == 0.6){
    return 1;
  } else {

  }
}
/*
//For debugging things
void printOnScreen(){
	//lcd::print(1, "Velocity FL: %f", FrontLeft.get_actual_velocity());
	//lcd::print(2, "Target Velocity FL: %f", drive.wheelTL);
  pros::lcd::print(0, "Inertial Reading: %f", inertial.get_rotation());
  pros::lcd::print(1, "Y Wheel Reading: %f", ((double) yWheel.get_value()));
  pros::lcd::print(2, "X Wheel Reading: %f", ((double) xWheel.get_value()));
}

void driverControl(double l, double r){
  //Calculates speed of wheels for driver control

	FrontLeft.move_velocity(l);
	FrontRight.move_velocity(r);
	BackLeft.move_velocity(l);
	BackRight.move_velocity(r);
}


void stopDrive(bool hold = false){
  //Shortcut to stop the drive quickly
  if(hold){
    FrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    FrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    BackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    BackRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  FrontLeft.move_velocity(0);
	FrontRight.move_velocity(0);
	BackLeft.move_velocity(0);
	BackRight.move_velocity(0);
  delay(100);
  FrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  FrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  BackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  BackRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
*/
