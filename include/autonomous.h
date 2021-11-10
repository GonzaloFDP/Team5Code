#include "../include/main.h"
#include "../include/functions.h"

#define MAXVELOCITY 100

double chassiskP = 0.001;
double chassiskI = 0.0;
double chassiskD = 0.0001;
std::shared_ptr<ChassisController> driveAuton = ChassisControllerBuilder()
    .withMotors({1,11},{-10,-20})
    .withGains({chassiskP, chassiskI, chassiskD}, {chassiskP,chassiskI, chassiskD}).withMaxVelocity(200)
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{9.75_in, 14.5_in}, imev5GreenTPR})
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

  /*driveAuton->moveDistance(7_in);
  goalController->setTarget(-250);
  driveAuton->turnAngleAsync(30_deg);
  Conveyor.move_velocity(200);
  driveAuton->waitUntilSettled();
  Conveyor.move_velocity(0);
  driveAuton->moveDistanceAsync(-10_in);*/

  profileController -> generatePath({
    {0_ft, 0_ft, 0_deg},
    {1_ft, 0_ft, 0_deg}},
    "A" //starting position
  );

  profileController -> generatePath({
    {0_ft, 0_ft, 0_deg}, // next point
    {0_ft, 0_ft, 90_deg}, // next point
    {1_ft, 0_ft, 90_deg}}, // next point
    "B" // Profile name
  );

  profileController->setTarget("B",true);
  profileController->waitUntilSettled();
  /*
  goalController->setTarget(-170);
  goalController->waitUntilSettled();
  ringtakeController->setTarget(150);
  profileController->setTarget("B",true);
  profileController->waitUntilSettled();
  ringtakeController->setTarget(0);*/










}

void Blue1(){
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {2_ft,0_ft,0_deg}},
    "C"
  );

  profileController->setTarget("C");
  pros::delay(2000);

}

void Blue2(){

}
