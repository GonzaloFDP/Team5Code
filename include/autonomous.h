#include "../include/main.h"
#include "../include/functions.h"

#define MAXVELOCITY 100





std::shared_ptr<AsyncPositionController<double,double>> goalController =
  AsyncPosControllerBuilder().withMotor(5).build();

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
  double chassiskP = 0.001;
  double chassiskI = 0.0;
  double chassiskD = 0.0001;
  std::shared_ptr<ChassisController> driveAuton = ChassisControllerBuilder().withMotors(-20, 11, -10, 1)
      .withGains({chassiskP, chassiskI, chassiskD}, {chassiskP,chassiskI, chassiskD}).withMaxVelocity(200)
      // Green gearset, 4 in wheel diam, 11.5 in wheel track
      .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 14.5_in}, imev5GreenTPR})
      .withOdometry()
      .buildOdometry();

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
  .withLimits({
    1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s)
    })
    .withOutput(driveAuton)
    .buildMotionProfileController();

  driveAuton->moveDistance(7_in);
  goalController->setTarget(-120);
  driveAuton->turnAngleAsync(30_deg);
  Conveyor.move_velocity(200);
  driveAuton->waitUntilSettled();
  Conveyor.move_velocity(0);
  driveAuton->moveDistanceAsync(-10_in);



}

void Blue1(){

}

void Blue2(){

}
