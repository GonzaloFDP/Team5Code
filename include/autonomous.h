#include "../include/main.h"
#include "../include/functions.h"

#define MAXVELOCITY 100

/*
std::shared_ptr<AsyncPositionController<double, double>> liftControl =
    AsyncPosControllerBuilder().withMotor(PBPort).build();
std::shared_ptr<AsyncPositionController<double, double>> fourbar =
    AsyncPosControllerBuilder().withMotor({FBRPort,-FBLPort}).build();
*/
//This file has all of the autonomous
void disabledAuton(){

}

void pop(){

}

void REDX(){/*
  std::shared_ptr<ChassisController> driveauton =
  ChassisControllerBuilder()
.withMotors({FLPort,BLPort},{FRPort,BRPort})
.withGains(
{0.0025, 0, 0.0001}, // Distance controller gains
{0.001, 0, 0.0001} // Turn controller gains
 )
// green gearset, 4 inch wheel diameter, 11.5 inch wheel track
.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
.withOdometry() // use the same scales as the chassis (above)
.buildOdometry(); // build an odometry chassis


driveauton->moveDistance(40_in);
Clamp.move_relative(1000, 100);
delay(1000);
driveauton->moveDistance(-40_in);
*/
}
void skills(){

}

void Red1(){
 FRmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 FLmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 BLmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 BRmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 pros::delay(2000);
 Clamp.move_relative(400, MAXVELOCITY);
 Conveyor.move_velocity(MAXVELOCITY);
 pros::delay(2000);
 Conveyor.set_brake_mode(MOTOR_BRAKE_COAST);
 Conveyor.move_velocity(0);
 FRmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
 FLmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
 BLmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
 BRmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
}

void Red2(){

}

void Blue1(){

}

void Blue2(){

}
