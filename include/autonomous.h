#include "../include/main.h"
#include "../include/functions.h"

#define MAXVELOCITY 100

/*double chassiskP = 0.001;
double chassiskI = 0.0;
double chassiskD = 0.0001;*/
std::shared_ptr<ChassisController> driveAuton = ChassisControllerBuilder()
    .withMotors({FL_MOTOR,BL_MOTOR,ML_MOTOR},{FR_MOTOR,BR_MOTOR,MR_MOTOR})
    .withGains(
    {0.0021, 0.0, 0.0001}, //distance gains
    {0.0027,0.0, 0.0001} //turn gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
      std::make_unique<AverageFilter<3>>()
    )
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4.88_in, 14.5_in}, imev5GreenTPR})
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



/*void test2(){
  profileController->generatePath({
    {0_in,0_in,0_deg},
    {12_in,0_in,0_deg}},
    "test1step3"
  );
  profileController->generatePath({
    {0_in,0_in,0_deg},
    {8_in,0_in,0_deg}},
    "test1step4"
  );
  profileController->setTarget("test1step3");
  pros::delay(1000);
  clampPiston.set_value(false);
  pros::delay(40);
  Fourbar.move_relative(700,100);
  pros:: delay(600);
  profileController->setTarget("test1step3",true);
  pros::delay(1000);
   driveAuton->turnAngle(-80_deg);
   pros::delay(8);
   profileController->setTarget("test1step3");
   pros::delay(1000);

  // move forward 12 inches then move backwards 8 inches and trun 90 degrees after the first move 8 inches then clamp down on the goal move forward 12 inches

}
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

  }*/

  void leftSideWPNoRingtake(){

    //left side WP only
    //done

    profileController -> generatePath({
      {0_in,0_in,0_deg},
      {16_in,0_in,0_deg}},
      "leftWP_step_1"
    );
    Fourbar.move_relative(1600, 100);
    pros::delay(300);
    profileController->setTarget("leftWP_step_1");
    pros::delay(1200);
    clampPiston.set_value(true);
    pros::delay(600);
    profileController->setTarget("leftWP_step_1",true);
    pros::delay(1200);
  }





  void test(){
  //  Clamp.move_relative(260, 100);
    pros::delay(200);
  }

  void rightSideWPNoRingtake(){

    /*driveAuton->moveDistance(7_in);
    goalController->setTarget(-250);
    driveAuton->turnAngleAsync(30_deg);
    Conveyor.move_velocity(200);
    driveAuton->waitUntilSettled();
    Conveyor.move_velocity(0);
    driveAuton->moveDistanceAsync(-10_in);*/
    profileController -> generatePath({
      {0_ft, 0_ft, 0_deg},
      {19_in, 0_ft, 0_deg}},
      "Q1_step_1" //starting position
    );
    profileController -> generatePath({
      {0_ft, 0_ft, 0_deg},
      {28_in, 0_ft, 0_deg}},
      "Q1_step_2" //starting position
    );

    profileController->setTarget("Q1_step_1", true);
    pros::delay(200);
    //Fourbar.move_relative(1050,75);
    pros::delay(500);
    //Fourbar.move_relative(-1050,75);
    pros::delay(600);
    Forklift.move_relative(1500, -100);
    pros::delay(800);
    profileController->setTarget("Q1_step_2"); //move towads neumogo
    pros::delay(2000);
    Forklift.move_relative(-1000, 100);
    pros::delay(500);

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

void officialSkills(){
  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {15_in,0_in,0_deg}},
    "Skills1"
  );
  profileController->setTarget("Skills1");
  pros::delay(1000);
  //Clamp.move_relative(1100,100);
  pros::delay(600);
  Forklift.move_relative(4250,100);
  pros::delay(800);
  driveAuton->turnAngle(208_deg);
  pros::delay(400);
  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {130_in,0_in,0_deg}},
    "Skills2"
  );
  profileController->setTarget("Skills2", true);
  pros::delay(1000);
  Fourbar.move_relative(1000,100);
  pros::delay(300);
  Forklift.move_relative(-1300,100);
  pros::delay(2200);
  driveAuton->turnAngle(130_deg);
  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {30_in,0_in,0_deg}},
    "Skills3"
  );
  pros::delay(1000);
  Fourbar.move_relative(2000,100);
  pros::delay(3000);
  driveAuton->turnAngle(-120_deg);
  profileController->setTarget("Skills3");
  driveAuton->turnAngle(-120_deg);
  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {15_in,0_in,0_deg}},
    "Skills4"
  );
  profileController->setTarget("Skills4");
  Fourbar.move_relative(-900,100);
  pros::delay(2000);
  //Clamp.move_relative(-1100,100);
  Fourbar.move_relative(900,100);
  pros::delay(2000);
  profileController->setTarget("Skills4",true);
  Fourbar.move_relative(-2000,100);

  //turn to right
  //grab neumogo
  //turn to left
  //place neumogo on plt4m and you lower forkLift
  //back towards oppo side (push tall mogo)
  //scoot forward a little and turn to the left
  //move towards neumogo
  //clamp it
  //turn 180 degrees
  //get amogo with forklift
  //move to other side of the field and board with 3 (one will not count)
}

void soloWP(){
  //solo WP (starts on left WP location)
  //not done

  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {14_in,0_in,0_deg}},
    "soloWP_step_1"
  );
  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {20_in,0_in,0_deg}},
    "soloWP_step_2"
  );
  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {155_in,0_in,0_deg}},
    "soloWP_step_3"
  );
  profileController -> generatePath({
    {0_in,0_in,0_deg},
    {65_in,0_in,0_deg}},
    "soloWP_step_4"
  );
  Fourbar.move_relative(1600, 100);
  pros::delay(300);
  profileController->setTarget("soloWP_step_1");
  pros::delay(1200);
  clampPiston.set_value(true);
  pros::delay(600);
  profileController->setTarget("soloWP_step_1",true);
  pros::delay(1200);
  driveAuton->turnAngle(101_deg);
  profileController->setTarget("soloWP_step_2",true);
  pros::delay(2000);
  driveAuton->turnAngle(97_deg);
  profileController->setTarget("soloWP_step_3", true);
  pros::delay(4000);
  Forklift.move_relative(1500,100);
  pros::delay(300);
  profileController->setTarget("soloWP_step_4");
  pros::delay(2000);

  //egg(omelet(benedict(poached(scambled(fried(sunnyside-up(wet(hard boiled))))))));
}

/*void leftSideForklift(){
  //done
  Clamp.move_relative(-1000,100);
  Fourbar.move_relative(100,100);
  profileController->generatePath({
    {0_ft,0_ft,0_deg},
    {65_in,0_ft,0_deg}},
    "LeftSide_step_1"
  );
  profileController->setTarget("LeftSide_step_1");
  pros::delay(600);
  Forklift.move_relative(degForForkLift,100);
  pros::delay(1300);
  Clamp.move_relative(1100, 100); //lower clamp
  pros::delay(300);
  Fourbar.move_relative(450,100);
  Forklift.move_relative(2000,100);
  pros::delay(1300); //wait until clamp is done
  driveAuton->turnAngle(-320_deg);
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
    {180_in,0_ft,0_deg}},
    "LeftSide_step_3"
  );
  profileController->setTarget("LeftSide_step_3");
  pros::delay(8000);
}*/



void leftSideNeumogo(){
  clampPiston.set_value(true);
  opDriver(200, 200);
//  Fourbar.move_relative(1330,75);
  pros::delay(500);
//  Fourbar.move_relative(-1500,75);
  pros::delay(600);
  FLmotor.move_velocity(0);
	FRmotor.move_velocity(0);
	BLmotor.move_velocity(0);
	BRmotor.move_velocity(0);
  MLmotor.move_velocity(0);
  MRmotor.move_velocity(0);
  clampPiston.set_value(false);
  pros::delay(200);
  Fourbar.move_relative(450,100);
  opDriver(-200,-200);
  pros::delay(1100);
  FLmotor.move_velocity(0);
	FRmotor.move_velocity(0);
	BLmotor.move_velocity(0);
	BRmotor.move_velocity(0);
  MLmotor.move_velocity(0);
  MRmotor.move_velocity(0);
}

void neumogoAndAWP(){
//  Fourbar.move_relative(1330,100);
  clampPiston.set_value(true);
  opDriver(175,175);
  pros::delay(500);
  //Fourbar.move_relative(-1430,100);
  pros::delay(580);
  FLmotor.move_velocity(0);
	FRmotor.move_velocity(0);
	BLmotor.move_velocity(0);
	BRmotor.move_velocity(0);
  MLmotor.move_velocity(0);
  MRmotor.move_velocity(0);
  clampPiston.set_value(false);
  pros::delay(200);
  Fourbar.move_relative(450,100);
  driveAuton->turnAngle(-40_deg);
  opDriver(-200,-200);
  pros::delay(700);
  Forklift.move_relative(1300,100);
  pros::delay(500);
  FLmotor.move_velocity(0);
	FRmotor.move_velocity(0);
	BLmotor.move_velocity(0);
	BRmotor.move_velocity(0);
  MLmotor.move_velocity(0);
  MRmotor.move_velocity(0);
}

void tallNeumogo(){
  clampPiston.set_value(true);
  opDriver(200, 200);
  pros::delay(340);
  //Fourbar.move_relative(1330,75);
  pros::delay(500);
//  Fourbar.move_relative(-1430,75);
  pros::delay(480);
  FLmotor.move_velocity(0);
	FRmotor.move_velocity(0);
	BLmotor.move_velocity(0);
	BRmotor.move_velocity(0);
  MLmotor.move_velocity(0);
  MRmotor.move_velocity(0);
  clampPiston.set_value(false);
  pros::delay(200);
  Fourbar.move_relative(450,100);
  opDriver(-200,-200);
  pros::delay(1000);
  FLmotor.move_velocity(0);
	FRmotor.move_velocity(0);
	BLmotor.move_velocity(0);
	BRmotor.move_velocity(0);
  MLmotor.move_velocity(0);
  MRmotor.move_velocity(0);
}
