#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Motor Init
	Motor LGB(LGPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor CL(CLPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor BL(BLPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor RGB(RGPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor CR(CRPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor BR(BRPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor LA(LAPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor RA(RAPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

	// Pneumatic init
	ADIDigitalOut tilt(tiltPort);
	ADIDigitalOut tiltClamp(tiltClampPort);
	ADIDigitalOut armClamp(armClampPort);

 	Task armControlTask(armControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Control Task");
	Task tilterTask(tiltControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tilter Task");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	Motor LGB(LGPort);
	Motor CL(CLPort);
	Motor BL(BLPort);
	Motor RGB(RGPort);
	Motor CR(CRPort);
	Motor BR(BRPort);
	Motor LA(LAPort);
	Motor RA(RAPort);
	ADIDigitalOut tilt(tiltPort);
	ADIDigitalOut tiltClamp(tiltClampPort);
	ADIDigitalOut armClamp(armClampPort);

	LGB.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	CL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	RGB.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	CR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	LA.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	RA.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	Controller master(E_CONTROLLER_MASTER);

	int armPos = 0;
	bool tankDrive = true;
	LA.tare_position();

	while(true) {
		double left, right;
		if(master.get_digital_new_press(DIGITAL_Y)) tankDrive = !tankDrive;

		if(tankDrive) {
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
		}else {
			double power = master.get_analog(ANALOG_LEFT_Y);
			double turn = master.get_analog(ANALOG_RIGHT_X);

			left = power + turn;
			right = power - turn;
		}

		LGB.move(left);
		CL.move(left);
		BL.move(left);
		RGB.move(right);
		CR.move(right);
 		BR.move(right);

		if (master.get_digital_new_press(DIGITAL_R2)){tiltSwitch();}
		if (master.get_digital_new_press(DIGITAL_R1)){armtiltSwitch();}
		if (master.get_digital_new_press(DIGITAL_L1)){
			armSwitch();
		}else if (master.get_digital_new_press(DIGITAL_L2)){
			scored();
		}
	}
}
