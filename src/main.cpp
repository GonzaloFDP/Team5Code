#include "../include/main.h"
#include  "../include/autonomous.h"


void initialize() {
	pros::lcd::initialize();
	screenPrintString(3, 2, "hiho");

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
     case 1:
		 	Red2();
     case 2:
		  Blue1();
     case 3:
		  Blue2();
     case 4:

     case 5:

     case 6:

     case 7:

		 case 8:
		 	skills();
   }
}


void opcontrol() {
	//okapi fancy stuff

	//Chassis Controller - lets us drive the robot around with open- or closed-loop Controller
	std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder()
		.withMotors(1,-10,11,20)
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
  while (true){
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);
		opDriver((power+turn)*plt4mMode, (power - turn)*plt4mMode);

		//Clamp
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			goalClampMovement(true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			goalClampMovement(false);
		} else {
			Clamp.set_brake_mode(MOTOR_BRAKE_HOLD);
			Clamp.move_velocity(0);
		}

		//Auton tester
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			autonomous();
		}

		//Ringtake
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			ringtakeMovement(true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			ringtakeMovement(false);
		} else {
			Ringtake.set_brake_mode(MOTOR_BRAKE_COAST);
			Ringtake.move_velocity(0);
		}

		//Conveyor
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			conveyorMovement(true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			conveyorMovement(false);
		} else {
			Conveyor.set_brake_mode(MOTOR_BRAKE_COAST);
			Conveyor.move_velocity(0);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			platformMode(plt4mMode);
		}

		screenPrintInt(2, 1, plt4mMode);

    pros::delay(20);
  }
}
