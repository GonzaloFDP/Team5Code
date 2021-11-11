#include "../include/main.h"
#include "../include/functions.h"

#define MAXVELOCITY 100

double chassiskP = 0.001;
double chassiskI = 0.0;
double chassiskD = 0.0001;
std::shared_ptr<ChassisController> driveAuton = ChassisControllerBuilder()
    .withMotors({FL_MOTOR,BL_MOTOR},{FR_MOTOR,BR_MOTOR})
    .withGains(
    {0.002, 0.0, 0.0001}, //distance gains
    {0.001,0.0, 0.0001} //turn gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
      std::make_unique<AverageFilter<3>>()
    )
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
  AsyncPosControllerBuilder()
  .withMotor(5)
  .build();

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

void Q1(){

  /*driveAuton->moveDistance(7_in);
  goalController->setTarget(-250);
  driveAuton->turnAngleAsync(30_deg);
  Conveyor.move_velocity(200);
  driveAuton->waitUntilSettled();
  Conveyor.move_velocity(0);
  driveAuton->moveDistanceAsync(-10_in);*/

  profileController -> generatePath({
    {0_ft, 0_ft, 0_deg},
    {2_ft, 0_ft, 0_deg}},
    "A" //starting position
  );

  profileController -> generatePath({
    {0_ft, 0_ft, 0_deg}, // next point
    {0_ft, 0_ft, 90_deg}, // next point
    {1_ft, 0_ft, 90_deg}}, // next point
    "B" // Profile name
  );

  profileController -> generatePath({
    {0_ft, 0_ft, 0_deg}, // next point
    {1_ft, 0_ft, 0_deg}, // next point
    {1_ft, 0_ft, -90_deg}}, // next point
    "C" // Profile name
  );

  profileController->setTarget("A",true);
  pros::delay(3000);
  goalController->setTarget(-170);
  goalController->waitUntilSettled();
  ringtakeController->setTarget(150);
  profileController->setTarget("B");
  pros::delay(800);
  ringtakeController->setTarget(0);
  goalController->setTarget(170);
  goalController->waitUntilSettled();
  profileController->setTarget("C", true);
  pros::delay(2000);
  goalController->setTarget(-170);
  goalController->waitUntilSettled();
  profileController->setTarget("A");
  pros::delay(2500);
  goalController->setTarget(170);
  goalController->waitUntilSettled();

}

void Q2(){
  profileController->generatePath({
    {0_in,0_ft,0_deg},
    {3_in,0_ft,0_deg}},
    "D"
  );

  profileController->generatePath({
    {5_in,0_ft,0_deg},
    {0_in,0_ft,0_deg},
    {0_in,0_ft,-90_deg}},
    "E"
  );

  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {3_ft,0_ft,0_deg}},
    "F"
  );

  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {2_ft,0_ft,0_deg}},
    "G"
  );


  profileController->setTarget("D",true);
  pros::delay(500);
  goalController->setTarget(-170);
  goalController->waitUntilSettled();
  ringtakeController->setTarget(150);
  pros::delay(900);
  ringtakeController->setTarget(0);
  goalController->setTarget(170);
  goalController->waitUntilSettled();
  profileController->setTarget("E");
  pros::delay(400);
  profileController->setTarget("F",true);
  pros::delay(1750);
  goalController->setTarget(-170);
  goalController->waitUntilSettled();
  profileController->setTarget("G");


}

void E1(){
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {3_ft,0_ft,0_deg}},
    "H"
  );

  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {2_ft,0_ft,0_deg},
    {2_ft,0_ft,90_deg}},
    "I"
  );

  profileController->setTarget("H",true);
  pros::delay(3000);
  goalController->setTarget(-170);
  goalController->waitUntilSettled();
  profileController->setTarget("I");
  pros::delay(3000);

}

void E2(){
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {3_ft,0_ft,0_deg}},
    "J"
  );

  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {2_ft,0_ft,0_deg},
    {2_ft,0_ft,90_deg}},
    "K"
  );

  profileController->setTarget("J",true);
  pros::delay(3000);
  goalController->setTarget(-170);
  goalController->waitUntilSettled();
  profileController->setTarget("K");
  pros::delay(3000);
}
