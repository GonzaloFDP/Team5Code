#include "../include/main.h"
#include "../include/functions.h"

#define MAXVELOCITY 100

double chassiskP = 0.001;
double chassiskI = 0.0;
double chassiskD = 0.0001;
std::shared_ptr<ChassisController> driveAuton = ChassisControllerBuilder()
    .withMotors({FL_MOTOR,BL_MOTOR},{FR_MOTOR,BR_MOTOR})
    .withGains(
    {0.0021, 0.0, 0.0001}, //distance gains
    {0.001,0.0, 0.0001} //turn gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
      std::make_unique<AverageFilter<3>>()
    )
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{2.17_in, 14.5_in}, imev5GreenTPR})
    .withOdometry()
    .buildOdometry();

std::shared_ptr<AsyncMotionProfileController> profileController =
AsyncMotionProfileControllerBuilder()
.withLimits({
  1.7, // Maximum linear velocity of the Chassis in m/s
    2.0, // Maximum linear acceleration of the Chassis in m/s/s
    10.0 // Maximum linear jerk of the Chassis in m/s/s/s)
  })
  .withOutput(driveAuton)
  .buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double,double>> goalController =
  AsyncPosControllerBuilder()
  .withMotor(5)
  .build();
//new auton - gets neutral and alliance goal (right side)
  void rightAutonNoRingtake(){
    profileController -> generatePath({
      {0_in,0_in,0_deg},
      {40_in,0_in,0_deg}},
      "auton_step_1"
    );
    profileController -> generatePath({
      {0_in,0_in,0_deg},
      {25_in,0_in,0_deg}},
      "auton_step_2"
    );
    //auton step 1
    profileController->setTarget("auton_step_1");
    pros::delay(2000);
    //get neutral goal with fourbar
    Forklift.move_relative(degForForkLift+1250,65);
    pros::delay(1700);
    Forklift.move_relative(degForForkLiftUp, 100);
    pros::delay(1000);
    //turn to the left
    driveAuton->turnAngle(-45_deg);
    pros::delay(1200);
    //lower clamp
    profileController->setTarget("auton_step_2",true);
    pros::delay(1300);
    Clamp.move_relative(-degForGoalClamp, 100);
    pros::delay(1700);
    //auton step 2
    driveAuton->turnAngle(45_deg);
    pros::delay(1200);
    profileController->setTarget("auton_step_2",true);
    pros::delay(1300);
    Clamp.move_relative(degForGoalClamp, 100);
    pros::delay(1600);

  }

  void leftSideWPNoRingtake(){

    //left side WP only

    profileController -> generatePath({
      {0_in,0_in,0_deg},
      {12_in,0_in,0_deg}},
      "Q3_step_1"
    );

    profileController->setTarget("Q3_step_1");
    pros::delay(700);
    Clamp.move_relative(1030,100);
    Clamp.move_relative(1030,100);
    pros::delay(700);
    Clamp.move_relative(-1030,100);
    Clamp.move_relative(-1030,100);
    pros::delay(700);
    profileController->setTarget("Q3_step_1",true);
    pros::delay(700);
  }



  void rightSideWPRingtake(){

    /*driveAuton->moveDistance(7_in);
    goalController->setTarget(-250);
    driveAuton->turnAngleAsync(30_deg);
    Conveyor.move_velocity(200);
    driveAuton->waitUntilSettled();
    Conveyor.move_velocity(0);
    driveAuton->moveDistanceAsync(-10_in);*/
    profileController -> generatePath({
      {0_ft, 0_ft, 0_deg},
      {34_in, 0_ft, 0_deg}},
      "Q1_step_1" //starting position
    );



    profileController->setTarget("Q1_step_1"); //move towards alliance goal
    pros::delay(950); // wait 800 ms
    Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
    pros::delay(1600); //wait until clamp is done
    conveyorController->setTarget(-165);
    profileController->setTarget("Q1_step_1",true); //move towads neumogo
    pros::delay(3000);
    conveyorController->setTarget(0); //stop conveyor
    Clamp.move_relative(degForGoalClamp,200);
    pros::delay(1800);

  }

  /*void Q2(){


    profileController->generatePath({
      {0_in,0_ft,0_deg},
      {3_in,0_ft,0_deg}},
      "Q2_step_1"
    );

    profileController->setTarget("Q2_step_1");
    pros::delay(700);
    Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
    pros::delay(1000); //wait until clamp is done
    //conveyorController->setTarget(200); //start conveyor at max speed
    //pros::delay(1500);
    //conveyorController->setTarget(0);
    Clamp.move_relative(degForGoalClamp, 200); //lower clamp
    pros::delay(3200); //wait until clamp is done
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {1_ft,0_ft,0_deg}},
      "Q2_step_3"
    );
    profileController->setTarget("Q2_step_3",true);
    pros::delay(1500);

  }*/

  void E1(){

    //left side neumogo DONT USE

    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {93_in,0_ft,0_deg}},
      "E1_step_1"
    );
    profileController->setTarget("E1_step_1");
    pros::delay(2400);
    Clamp.move_relative(-1900, 200); //lower clamp
    pros::delay(900); //wait until clamp is done
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {70_in,0_ft,0_deg}},
      "E1_step_2"
    );
    profileController->setTarget("E1_step_2",true);
    pros::delay(1500);
    driveAuton->turnAngle(240_deg);
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {20_in,0_ft,0_deg}},
      "E1_step_3"
    );
    profileController->setTarget("E1_step_3");
    pros::delay(1800);
    Clamp.move_relative(1900, 200);
    pros::delay(600);
    profileController->generatePath({
      {0_in,0_ft,0_deg},
      {50_in,0_ft,0_deg}},
      "E1_step_4"
    );
    profileController->setTarget("E1_step_4",true);
    pros::delay(1800);
    driveAuton->turnAngle(-120_deg);
    //turn -90 degrees
    profileController->setTarget("E1_step_4");
    pros::delay(1850);
    //move e
    Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
    pros::delay(900); //wait until clamp is done
    //clamp alliance
    //conveyorController->setTarget(-165);
    //pros::delay(1);
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {42_in,0_ft,0_deg}},
      "E1_step_6"
    );
    profileController->setTarget("E1_step_6",true); //move towads neumogo
    pros::delay(2500);
    //conveyorController->setTarget(0); //stop conveyor
    //pros::delay(1);
    //start conveyor and hood
    Clamp.move_relative(degForGoalClamp, 200); //lower clamp
    pros::delay(900);

    //clear awp line
    //stop conveyor and hood
    //release goal


  }

  void rightSideNeumogoWPRingtake(){

    //right side neumogo and AWP OLD
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {83_in,0_ft,0_deg}},
      "E2_step_1"
    );
    profileController->setTarget("E2_step_1");
    pros::delay(2000);
    Clamp.move_relative(-2100, 200); //lower clamp
    pros::delay(850); //wait until clamp is done
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {60_in,0_ft,0_deg}},
      "E2_step_2"
    );
    profileController->setTarget("E2_step_2",true);
    pros::delay(1800);
    driveAuton->turnAngle(170_deg);
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {25_in,0_ft,0_deg}},
      "E2_step_3"
    );
    profileController->setTarget("E2_step_3");
    pros::delay(1500);
    Clamp.move_relative(1900, 200);
    pros::delay(600);
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {20_in,0_ft,0_deg}},
      "E2_step_4"
    );
    profileController->setTarget("E2_step_4",true);
    pros::delay(1500);
    driveAuton->turnAngle(-90_deg);
    //turn -90 degrees
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {46_in,0_ft,0_deg}},
      "E2_step_4"
    );
    profileController->setTarget("E2_step_4");
    pros::delay(2000);
    //move e
    Clamp.move_relative(-degForGoalClamp, 200); //lower clamp
    pros::delay(800); //wait until clamp is done
    //clamp alliance
    conveyorController->setTarget(-165);
    profileController->generatePath({
      {0_ft,0_ft,0_deg},
      {32_in,0_ft,0_deg}},
      "E2_step_6"
    );
    profileController->setTarget("E2_step_6",true); //move towads neumogo
    pros::delay(1600);
    conveyorController->setTarget(0); //stop conveyor
    //start conveyor and hood
    Clamp.move_relative(degForGoalClamp, 200); //lower clamp
    pros::delay(900);
  }

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
void skillsNoBoardingLeftSide(){
  // get the red goal with the lift clamp
  // turn left so forklift side is facing the rest of field
  // go foward and push neutral goal with the forklift side to the opposing home zone wall and back up a bit - 20 pts
  // turn right and drop red alliance goal - 20 pts
  // turn left (in terms of traction wheels)
  // push blue goal with forklift side to original home zone - 20 pts
  // turn left (in terms of traction wheels)
  // grab neutral goal with goal clamp
  // turn left and open forkLift
  // turn right and push neutral goal
  // move to blue home zone and drop neutral goal in corner while backing out forkLift - 20 pts
  // lift forklift up while turning right 180 degrees so goal clamp faces where forklift was
  // turn right, move forward, and grab blue goal with goal clamp - 20 pts
  // move backwards, turn 180 degrees and drop alliance goal - 20 pts
  // turn left, grab last alliance goal with goal clamp, turn left again, move straight, and drop goal in other home zone - 20 pts
}
void skillsBoardingRightSide(){
  // start facing red alliance goal on blue home zone
  // clamp alliance goal
  // move left so forklift (closed) facing the neutral goal
  // move backward so forklift (closed) pushes neutral mobile goal into home zone - 20 pts
  // move forward a bit
  // move 270 degrees right so the side with the alliance goal clamped is facing to the left of the neutral goal
  // move backwards and drop alliance goal - 20 pts
  // move left and align with middle tall neutral mogo
  // move backwards with closed forklift so neutral mobile gets pushed into blue home zone - 20 pts
  // turn right to clamp on smol neutral mobile goal
  // turn right to face blue home zone
  // align diagonally to end tile to board
  // move forward so goal is in corner
  // move left to align with paltform
  // board on half speed platform mode
}
void leftSideForklift(){
  //done
  Clamp.move_relative(-1000,100);
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {125_in,0_ft,0_deg}},
    "LeftSide_step_1"
  );
  profileController->setTarget("LeftSide_step_1");
  pros::delay(600);
  Forklift.move_relative(degForForkLift,100);
  pros::delay(1900);
  Clamp.move_relative(degForGoalClamp, 100); //lower clamp
  pros::delay(500);
  Fourbar.move_relative(450,100);
  Forklift.move_relative(1850,100);
  pros::delay(1300); //wait until clamp is done
  driveAuton->turnAngle(-132_deg);
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {60_in,0_ft,0_deg}},
    "LeftSide_step_2"
  );
  profileController->setTarget("LeftSide_step_2",true);
  pros::delay(2300);
  Forklift.move_relative(-1000,80);
  pros::delay(900);
  driveAuton->turnAngle(-90_deg);
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {140_in,0_ft,0_deg}},
    "LeftSide_step_3"
  );
  profileController->setTarget("LeftSide_step_3");
  pros::delay(6700);
}

void middleMogo(){
  profileController->generatePath({
    {0_in, 0_in, 0_deg},
    {60_in,0_in,0_deg}},
    "middleMogo step 1"
  );
  profileController->setTarget("middleMogo step 1");
}
void Red1(){
 FRmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 FLmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 BLmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 BRmotor.move_relative(distanceToTicks(20), MAXVELOCITY);
 pros::delay(2000);
 Clamp.move_relative(400, MAXVELOCITY);
 //Conveyor.move_velocity(MAXVELOCITY);
 pros::delay(2000);
 //Conveyor.set_brake_mode(MOTOR_BRAKE_COAST);
 //Conveyor.move_velocity(0);
 FRmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
 FLmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
 BLmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
 BRmotor.move_relative(distanceToTicks(20), -1*MAXVELOCITY);
}
