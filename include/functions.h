#include "../include/main.h"
#include "../include/motorSetup.h"

//okapi::Controller master;

std::shared_ptr<AsyncVelocityController<double,double>> conveyorController =
  AsyncVelControllerBuilder().withMotor(CONVEYOR).build();

std::shared_ptr<AsyncVelocityController<double,double>> ringtakeController =
  AsyncVelControllerBuilder().withMotor(RINGTAKE).build(); //? idk why this doesn't work

//Useful Constants
const double wheelCircumfrence = 3.25 * M_PI;

const int degForGoalClamp = 740;

const int degForForkLift = 2000;

const int degForForkLiftUp = 2000;

int countr = 0;
std::string autons[8] = {"Red1", "rightSideWPRingtake", "leftSideWPNoRingtake", "rightSideNeumogoWPRingtake", "leftSideForklift", "Disabled", "forkLiftThing","Q3"};
int size = 8;//*(&autons + 1) - autons;

void screenPrintString(int e, int o, std::string i){
  master.print(e,o,"%s",i.c_str());
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
    Clamp.move_velocity(100);//down
  } else if (upOrDown == false){
    Clamp.move_velocity(-100);//up
  } else {
    Clamp.move_velocity(0);
  }
}

void ringtakeMovement(bool upOrDown){
  if (upOrDown){
    ringtakeController->setTarget(100);
  } else if (upOrDown == false){
    ringtakeController->setTarget(100);
  } else {
    ringtakeController->setTarget(0);
  }
}

void conveyorMovement(bool upOrDown){
  if (upOrDown){
    conveyorController->setTarget(100);
  } else if (upOrDown == false){
    conveyorController->setTarget(100);
  } else {
    conveyorController->setTarget(0);
  }
}

void fourBarMovement(bool upOrDown){
  if (upOrDown){
    Fourbar.move_velocity(100); //up
  } else if (upOrDown == false){
    Fourbar.move_velocity(-100); //down
  } else {
    Fourbar.move_velocity(0);
  }
}

// fork lift
void forkLiftMovement(bool upOrDown){
  if(upOrDown){
    Forklift.move_voltage(7000);
  }
  else if(upOrDown == false){
    Forklift.move_voltage(-7000);
  }
  else{
    Forklift.move_voltage(0);
  }
}

/*
//For debugging things
voisd printOnScreen(){
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
