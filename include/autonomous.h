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
    {31_in, 0_ft, 0_deg}},
    "31Inches" //starting position
  );

  profileController -> generatePath({
    {0_ft, 0_ft, 0_deg},
    {28_in, 0_ft, 0_deg}},
    "20Inches" //starting position
  );

  profileController->generatePath({
    {0_in, 0_ft, 0_deg},
    {5_in,0_ft,0_deg}},
    "5inches"
  );

  profileController->setTarget("31Inches"); //move towards alliance goal
  pros::delay(810); // wait 800 ms
  Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
  pros::delay(1000); //wait until clamp is done
  conveyorController->setTarget(200); //start conveyor at max speed
  //turn 90 degrees left
  pros::delay(800); //wait until done turning
  conveyorController->setTarget(0); //stop conveyor
  profileController->setTarget("20Inches",true); //move towads neumogo
  pros::delay(2500);
  Clamp.move_relative(degForGoalClamp, 200); //raise clamp
  pros::delay(2400);
  profileController->setTarget("5inches",true);
  pros::delay(2500);

}

void Q2(){
  profileController->generatePath({
    {0_in,0_ft,0_deg},
    {3_in,0_ft,0_deg}},
    "D"
  );

  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {3_ft,0_ft,0_deg}},
    "F"
  );

  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {1_ft,0_ft,0_deg}},
    "G"
  );


  profileController->setTarget("D");
  pros::delay(700);
  Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
  pros::delay(1000); //wait until clamp is done
  conveyorController->setTarget(200); //start conveyor at max speed
  pros::delay(1500);
  conveyorController->setTarget(0);
  Clamp.move_relative(degForGoalClamp, 200); //lower clamp
  pros::delay(3200); //wait until clamp is done
  profileController->setTarget("G",true);
  pros::delay(1500);

}

void E1(){
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {84_in,0_ft,0_deg}},
    "H"
  );


  profileController->setTarget("H");
  pros::delay(2000);
  Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
  pros::delay(1000); //wait until clamp is done
  profileController->setTarget("H",true);
  pros::delay(2000);


}

void E2(){
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {42_in,0_ft,0_deg}},
    "J"
  );

  profileController->setTarget("J");
  pros::delay(1600);
  Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
  pros::delay(3200); //wait until clamp is done
  profileController->setTarget("J",true);
  pros::delay(1500);
}
