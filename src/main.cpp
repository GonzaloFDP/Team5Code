#include "../include/main.h"
#include  "../include/autonomous.h"


void initialize() {
	pros::lcd::initialize();
	//inertial.reset();
	screenPrintString(2, 2, "g");
	FLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	FRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	BLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	BRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	MLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	MRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	Fourbar.set_brake_mode(MOTOR_BRAKE_HOLD);
	Forklift.set_brake_mode(MOTOR_BRAKE_HOLD);

  autonSelector();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  switch(countr){
     case 0:
		 	rightSideWPNoRingtake();
			break;
     case 1:
		 	//test2();
			break;
     case 2:
		  leftSideWPNoRingtake();
			break;
     case 3:
		  tallNeumogo();
			break;
     case 4:
		 	//leftSideForklift();
			break;
     case 5:
		 	test();
			break;
     case 6:
		 	leftSideNeumogo();
			break;
     case 7:
		  neumogoAndAWP();
			break;
		 case 8:
		 	tallNeumogo();
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
	FLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	FRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	BLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	BRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	MLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
	MRmotor.set_brake_mode(MOTOR_BRAKE_COAST);

	double plt4mMode = 1;
	master.clear();
	Forklift.set_brake_mode(MOTOR_BRAKE_HOLD);

  while (true){
		//screenPrintInt(0,1,plt4mMode);
		screenPrintDub(0,1,inertial.get_accel().x);

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			if(plt4mMode == 1){
				plt4mMode = 0.4;
				FLmotor.set_brake_mode(MOTOR_BRAKE_HOLD);
				FRmotor.set_brake_mode(MOTOR_BRAKE_HOLD);
				BLmotor.set_brake_mode(MOTOR_BRAKE_HOLD);
				BRmotor.set_brake_mode(MOTOR_BRAKE_HOLD);
				MLmotor.set_brake_mode(MOTOR_BRAKE_HOLD);
				MRmotor.set_brake_mode(MOTOR_BRAKE_HOLD);
			} else if (plt4mMode == 0.4){
				plt4mMode = 1;
				FLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
				FRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
				BLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
				BRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
				MLmotor.set_brake_mode(MOTOR_BRAKE_COAST);
				MRmotor.set_brake_mode(MOTOR_BRAKE_COAST);
		}
	}
// fork lift
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			forkLiftMovement(false);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			forkLiftMovement(true);
		} else {
			Forklift.move_voltage(0);
		}

	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
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
			//goalClampMovement(false);
			clampPiston.set_value(true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			//goalClampMovement(true);
			clampPiston.set_value(false);
		} else {

		}



		//Auton tester
	/*	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			autonomous();
		}*/
		pros::delay(20);
	}
}
