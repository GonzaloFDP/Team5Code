#include "../include/main.h"
#include  "../include/autonomous.h"


void initialize() {
//	ForkliftLeft.move_relative(degForForkLift,100);
	//ForkliftRight.move_relative(degForForkLift,100);
	pros::lcd::initialize();
	screenPrintString(2, 2, "o");

	//pros::lcd::register_btn0_cb(leftBtn);
	//pros::lcd::register_btn1_cb(centerBtn);
	//pros::lcd::register_btn2_cb(rightBtn);

	Clamp.set_brake_mode(MOTOR_BRAKE_HOLD);
	//Fourbar.set_brake_mode(MOTOR_BRAKE_HOLD);
  autonSelector();

	//autonSelector();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  switch(countr){
     case 0:
		 	Red1();
			break;
     case 1:
		 	rightSideWPRingtake();
			break;
     case 2:
		  leftSideWPNoRingtake();
			break;
     case 3:
		  rightSideNeumogoWPRingtake();
			break;
     case 4:
		 	leftSideForklift();
			break;
     case 5:
		 	disabledAuton();
			break;
     case 6:
		 	leftSideForklift();
			break;
     case 7:
		  skillsNoBoardingLeftSide();
			break;
		 case 8:
		 	skillsBoardingRightSide();
			break;
   }

}

//void rings(void* param) {
	//ringtakeMovement(param);
	//pros::delay(10);
//}

//void conveyor(void* param){
//	conveyorMovement(param);
	//pros::delay(10);
//}

/*void my_task_fn(void* param) {
	std::string t = std::to_string( (FLmotor.get_temperature()+FRmotor.get_temperature() + BLmotor.get_temperature()+ BRmotor.get_temperature()+Clamp.get_temperature()+Ringtake.get_temperature()+ Conveyor.get_temperature())/7);
	screenPrintString(1, 1, t);
	pros::delay(200);
}*/

void my_task_fn(void* param) {
	std::string dt ="Drivetrain" + std::to_string( (FLmotor.get_temperature()+FRmotor.get_temperature() + BLmotor.get_temperature()+ BRmotor.get_temperature())/4);
	std::string lift ="Lift" + std::to_string( (Fourbar.get_temperature()));
	screenPrintString(1, 1, dt.c_str());
	screenPrintString(2, 1, lift.c_str());
		pros::delay(200);
}

void opcontrol() {
	master.clear();
	//autonomous();
	//okapi fancy stuff

	//Chassis Controller - lets us drive the robot around with open- or closed-loop Controller
	std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder()
		.withMotors(1,-10,11,-20)
//haha funny meme
		//Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
		.build();
/*-------------------------------------------Tank-Okapi---------------------------------------------------------------------

		//Joystick values -1 -> 1
		Controller controller;
		while(true){
			drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
			pros::delay(10);
		}*/

/*-------------------------------------------Arcade-Okapi-------------------------------------------------------------------

		//Joystick values -1 -> 1
		Controller controller;
		while(true){
		drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		pros::delay(10);
	}*/

/*-------------------------------------------Arm-Control-------------------------------------------------------------------

	ADIButton armLimitSwitch('H');
	Motor armMotor(-8);
	ControllerButton armUpButton(ControllerDigital::A);
	ControllerButton armDownButton(ControllerDigital::B);
	ControllerButton autonButton(ControllerDigital::X);
	//Switch control
	if(armLimitSwitch.isPressed()) {
		armMotor.moveVelocity(0);
	} else {
		//Just keep going idk man
		if (armUpButton.isPressed()){
			armMotor.moveVoltage(12000);
		} else if (armDownButton.isPressed()){
			armMotor.moveVoltage(-12000);
		} else {
			armMotor.moveVoltage(0);
		}
	}

	//haha funnie auton stuff
	if(autonButton.changedToPressed()){
		drive->moveDistance(20_in);
		drive->turnAngle(90_deg);
	}*/

	double plt4mMode = 1;
	master.clear();
	Clamp.set_brake_mode(MOTOR_BRAKE_HOLD);

//	bool ringMove;
//	bool convMove;

  while (true){
		screenPrintInt(0,1,plt4mMode);
		pros::Task my_task(my_task_fn);

//	pros::Task my_task(rings, (void*)ringMove, "ring");
//	pros::Task the_task(conveyor, (void*)convMove, "conveyor");

		//Conveyor Task

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
				conveyorController->setTarget(-167);
		} else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
				conveyorController->setTarget(167);
		} else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			conveyorController->setTarget(0);
			Conveyor.set_brake_mode(MOTOR_BRAKE_HOLD);
			Conveyor.move_velocity(0);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){ //remember to check ports; see if connected
			if(plt4mMode == 1){
				plt4mMode = 0.6;
			} else if (plt4mMode == 0.6){
				plt4mMode = 1;
		}
	}
// fork lift
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			forkLiftMovement(false);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			forkLiftMovement(true);
		} else {
			Forklift.set_brake_mode(MOTOR_BRAKE_HOLD);
			Forklift.move_voltage(0);
		}

	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ //remember to check ports; see if connected
		fourBarMovement(true); //down
	} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
		fourBarMovement(false); //up
	} else {
		Fourbar.set_brake_mode(MOTOR_BRAKE_HOLD);
		Fourbar.move_velocity(0);
	}

		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		opDriver(((power+turn)*1.6)*plt4mMode, ((power - turn)*1.6)*plt4mMode);

		//Clamp
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			goalClampMovement(false);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			goalClampMovement(true);
		} else {
			Clamp.set_brake_mode(MOTOR_BRAKE_HOLD);
			Clamp.move_velocity(0);
		}



		//Auton tester
	/*	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			autonomous();
		}*/
		pros::delay(20);
	}
}
