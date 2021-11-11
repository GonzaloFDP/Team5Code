#include "../include/main.h"
#include  "../include/autonomous.h"


void initialize() {
	pros::lcd::initialize();
	screenPrintString(2, 2, "q");

	//pros::lcd::register_btn0_cb(leftBtn);
	//pros::lcd::register_btn1_cb(centerBtn);
	//pros::lcd::register_btn2_cb(rightBtn);

	Clamp.set_brake_mode(MOTOR_BRAKE_HOLD);
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
		 	Q1();
			break;
     case 2:
		  Q2();
			break;
     case 3:
		  E1();
			break;
     case 4:
		 	E2();
			break;
     case 5:
		 	disabledAuton();
			break;
     case 6:

     case 7:

		 case 8:
		 	skills();
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

void opcontrol() {
	//autonomous();
	//okapi fancy stuff

	//Chassis Controller - lets us drive the robot around with open- or closed-loop Controller
	std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder()
		.withMotors(1,-10,11,-20)
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
	autonomous();

  while (true){

//	pros::Task my_task(rings, (void*)ringMove, "ring");
//	pros::Task the_task(conveyor, (void*)convMove, "conveyor");

		//Conveyor Task

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
				conveyorController->setTarget(150);
		} else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
				conveyorController->setTarget(-150);
		} else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			conveyorController->setTarget(0);
			Conveyor.set_brake_mode(MOTOR_BRAKE_HOLD);
			Conveyor.move_velocity(0);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){ //remember to check ports; see if connected
				ringtakeController->setTarget(-150);//                          to port 12
		} else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
				ringtakeController->setTarget(150);
		} else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			ringtakeController->setTarget(0);
			Conveyor.set_brake_mode(MOTOR_BRAKE_HOLD);
			Conveyor.move_velocity(0);
		}

		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);
		opDriver((power+turn)*plt4mMode, (power - turn)*plt4mMode);

		//Clamp
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			goalClampMovement(false);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			goalClampMovement(true);
		} else {
			Clamp.set_brake_mode(MOTOR_BRAKE_HOLD);
			Clamp.move_velocity(0);
		}

		//Auton tester
	/*	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			autonomous();
		}*/
	}
}
