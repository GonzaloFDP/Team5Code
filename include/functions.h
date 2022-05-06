#include "../include/main.h"
#include "../include/motorSetup.h"

//okapi::Controller master;

std::shared_ptr<AsyncVelocityController<double,double>> conveyorController =
  AsyncVelControllerBuilder().withMotor(CONVEYOR).build();

std::shared_ptr<AsyncVelocityController<double,double>> ringtakeController =
  AsyncVelControllerBuilder().withMotor(RINGTAKE).build(); //? idk why this doesn't work

//Useful Constants
const double wheelCircumfrence = 3.25 * M_PI * 1.5;

const int degForGoalClamp = 470;

const int degForForkLift = 2000;

const int degForForkLiftUp = 2000;

bool pidWait = false;

int fourbarHeight [3] = {0,1000,4000};

std::string egg = "egg";

int countr = 0;
std::string autons[9] = {"rightSideWPNoRingtake", "rightSideTwoGoal", "leftSideWPNoRingtake", "tallNeumogo", "soloWP", "canada", "leftSideNeumogo","neumogoWP","e"};
int size = 9;

void screenPrintString(int e, int o, std::string i){
  master.print(e,o,"%s",i.c_str());
}

void screenPrintInt(int e, int o, double i){
  master.print(e, o, "%i", int(i));
}

void screenPrintDub(int e, int o, double i){
  master.print(e, o, "%d", i);
}

double distanceToTicks(double inches){
  double ticks = inches;
  ticks /= (wheelCircumfrence);
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
       pros::delay(100);
       if(master.get_digital(DIGITAL_A)){
         break;
       }
     }
   }
   master.rumble("..");
}

void opDriver(double left, double right){
  //Calculates speed of wheels for driver control
  if (left > 200){
    left = 200;
  } else if (left<-200){
    left = -200;
  }

  if (right>200){
    right=200;
  } else if (right < -200){
    right = -200;
  }
	FLmotor.move_velocity(left);
	FRmotor.move_velocity(right);
	BLmotor.move_velocity(left);
	BRmotor.move_velocity(right);
  MLmotor.move_velocity(left);
	MRmotor.move_velocity(right);

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
    Forklift.move_velocity(100);
  }
  else if(upOrDown == false){
    Forklift.move_velocity(-100);
  }
  else{
    Forklift.move_velocity(0);
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



*/

bool resetDriveSensors;

void turnAngle(double angle){
  inertial.tare();
  int turnPrevError = 0;
  int turnDerivative;
  double turnKP = 0.003;
  double turnKD = 0.001;
  int initialT = pros::millis();
  int tVal = angle;
  int error = angle - inertial.get_rotation();
  pros::lcd::print(2, std::to_string(inertial.get_rotation()).c_str());
  while(error>5){
    if(resetDriveSensors){
      resetDriveSensors = false;
      inertial.tare();
    }

    error = tVal - inertial.get_rotation();

    turnDerivative = error - turnPrevError;

    double turnMotorPower = (error * turnKP + turnDerivative * turnKD);

    opDriver(turnMotorPower, -turnMotorPower);

    turnPrevError = error;
    pros::delay(20);
  }
}

void moveDistance(double distance, double kP, double kD, double timeMill){
  FLmotor.tare_position();
  FRmotor.tare_position();
  MLmotor.tare_position();
  MRmotor.tare_position();
  BLmotor.tare_position();
  BRmotor.tare_position();

  int prevError = 0;
  int derivative;
  double avgLeftSide = (FLmotor.get_position() + MLmotor.get_position() + BLmotor.get_position())/3;
  double avgRightSide = (FRmotor.get_position() + MRmotor.get_position() + BRmotor.get_position())/3;
  double totalAvgPos = (avgLeftSide+avgRightSide)/2;
  int tVal = distanceToTicks(distance);
  int error = tVal - totalAvgPos;
  int counter = 0;
  while(true){
    if(counter*20 >= timeMill){
      break;
    }
    avgLeftSide = (FLmotor.get_position() + MLmotor.get_position() + BLmotor.get_position())/3;
    avgRightSide = (FRmotor.get_position() + MRmotor.get_position() + BRmotor.get_position())/3;
    totalAvgPos = (avgLeftSide+avgRightSide)/2;
    error = tVal - totalAvgPos;
    derivative = error - prevError;
    double motorPower = (error * kP + derivative * kD);
    if(motorPower>200){
      motorPower = 200;
    } else if (motorPower<-200){
      motorPower = -200;
    } else {}
    opDriver(motorPower,motorPower);
    prevError = error;
    counter++;
    pros::delay(20);
  }
  FLmotor.move_velocity(0);
	FRmotor.move_velocity(0);
	BLmotor.move_velocity(0);
	BRmotor.move_velocity(0);
  MLmotor.move_velocity(0);
  MRmotor.move_velocity(0);
}

void checkTug(){
  if(inertial.get_accel().x > -0.1){
    Fourbar.move_relative(-450,100);
    clampPiston.set_value(true);
  }
}
