#include "main.h"
#include "autonSets.hpp"

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
	Imu Inertial(ImuPort);
	Inertial.reset();

	// Pneumatic init
	ADIDigitalOut tilt(tiltPort);
	ADIDigitalOut tiltClamp(tiltClampPort);
	ADIDigitalOut armClamp(armClampPort);

 	Task armControlTask(armControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Control Task");
	Task tilterTask(tiltControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tilter Task");
	Task debugTask(Debug, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
	Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	Task sensorsTask(Sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	Task controlTask(Control, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	Task armclampControlTask(armclampControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	CL.tare_position();
	CR.tare_position();
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
void autonomous() {
	double start = millis();

	// start of auton
		//grab 1st goal
	baseMove(-7);
	waitBase(1000);
	tiltSwitch();
	delay(1000);
	baseMove(18);
	waitBase(1000);
	baseTurn(95);
	waitBase(1000);
	baseMove(45);
	waitBase(2000);
	delay(10);
	armtiltSwitch();
	waitBase(1000);

		//score 1st goal
	baseTurn(118);
	delay(10);
	waitBase(1000);
	armState();
	baseMove(61, 0.48, 0.3);
	waitBase(2300);
	baseTurn(78);
	waitBase(3000);
	scoreState();
	delay(300);
	armtiltSwitch();
	delay(200);
	scoreState();
	baseMove(-10, 0.45, 0.4);

	// grab 2nd goal
	waitBase(1500);
	baseTurn(-102, 2, 0);
	waitBase(1500);
	tiltSwitch();
	armState();
	delay(600);
	baseMove(15, 0.4, 0.4);
	waitBase(1500);
	delay(10);
	armtiltSwitch();

	// score 2nd goal
	baseTurn(-133);
	waitBase(1500);
 	baseMove(44);
	waitBase(3000);
	baseTurn(-10);
	waitBase(2000);
	armtiltSwitch();

	// grab 3rd goal
	baseMove(-29);
	waitBase(2000);
	tiltSwitch();
	delay(10);
	baseMove(17);
	waitBase(2000);
	baseTurn(80);
	waitBase(2000);

	// grab 4th goal
	baseMove(23);
	waitBase(2000);
	delay(10);
	armtiltSwitch();
	waitBase(2000);
	armState();
	baseTurn(49);

	//score 4th goal
	waitBase(2000);
	baseMove(56);
	waitBase(2000);
	scoreState();
	delay(300);
	armtiltSwitch();
	delay(30);
	scoreState();

	baseMove(-10);
	waitBase(2000);
	armState();
	baseTurn(-9);
	waitBase(2000);
	baseMove(28);
	waitBase(2000);
	armtiltSwitch();
	delay(100);
	armState();
	delay(100);
	baseTurn(84);
	waitBase(2000);
	baseMove(5);
	waitBase(2000);
	delay(200);
	armtiltSwitch();
	delay(200);
	baseMove(-5);
	waitBase(2000);
	baseTurn(-9);
	waitBase(2000);
	// tiltSwitch();
	// delay(200);
	// baseTurn();
	// waitBase(2000);
	// baseMove();
	// waitBase(2000);
	// tiltSwitch();
	// delay(200);
	// baseMove();
	// waitBase();
	// armtiltSwitch();
	// delay(200);
	// armState();
	// delay(200);
	// baseTurn();
	// waitBase(2000);
	// scoreState();
	// delay(200);
	// armtiltSwitch();
	// delay(100);

	printf("program finished in %.2fs", millis() - start);
}

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
	Task armcontrolTask(armControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);

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
	bool preset = true;
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

		if (master.get_digital_new_press(DIGITAL_A)){preset = !preset;}

		if (preset){
			armcontrolTask.resume();
			if (master.get_digital_new_press(DIGITAL_R2)){tiltSwitch();}
			if (master.get_digital_new_press(DIGITAL_R1)){armtiltSwitch();}
			if (master.get_digital_new_press(DIGITAL_L1)){armState();}
			if (master.get_digital_new_press(DIGITAL_L2)){scoreState();}
			if (master.get_digital_new_press(DIGITAL_X)){tallestRings();}
		} else {
			armcontrolTask.suspend();
			if (master.get_digital_new_press(DIGITAL_R2)){tiltSwitch();}
			if (master.get_digital_new_press(DIGITAL_R1)){armtiltSwitch();}
			if(master.get_digital(DIGITAL_L1)){
				LA.move(127);
				RA.move(127);
			} else if (master.get_digital(DIGITAL_L2)){
				LA.move(-127);
				RA.move(-127);
			} else {
				LA.move(0);
				RA.move(0);
			}
		}
	}
}
